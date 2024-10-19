use crate::sl::cmd::ScanModeConfEntry::*;
use crate::sl::cmd::SlLidarCmd::{
    GetDeviceHealth, GetDeviceInfo, GetLidarConf, GetSampleRate, HQMotorSpeedCtrl, Reset, Scan,
    Stop,
};
use crate::sl::cmd::{
    ScanModeConfEntry, SlLidarResponseDeviceHealthT, SlLidarResponseDeviceInfoT,
    SlLidarResponseGetLidarConf, SlLidarResponseSampleRateT,
};
use crate::sl::error::RxError;
use crate::sl::error::RxError::Corrupted;
use crate::sl::{Channel, Response, ResponseDescriptor};
use serialport::SerialPort;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{mpsc, Arc};
use std::thread;
use std::thread::sleep;
use std::time::Duration;

const S1_BAUD: usize = 256000;

#[derive(Debug, Clone)]
enum LidarState {
    Idle,
    Processing,
    Scanning,
    ProtectionStop,
}

#[derive(Debug, Clone)]
pub struct Sample {
    pub(crate) start: bool,
    pub(crate) intensity: u8,
    pub(crate) angle: u16,
    pub(crate) distance: u16,
}

/// Represents a serial connection to a lidar
pub struct Lidar {
    /// State of the lidar's internal state machine
    nuke: Arc<AtomicBool>,
    /// serial connection object
    channel: Box<dyn serialport::SerialPort>,

    /// reader thread handle
    thread_handle: Option<thread::JoinHandle<()>>,
}

impl Lidar {
    /// initializes a serial connection to the lidar on the given port.
    pub fn init(port: String) -> Result<Lidar, serialport::Error> {
        serialport::new(&port, S1_BAUD as u32)
            .timeout(Duration::from_millis(1000))
            .open()
            .map(|channel| Lidar {
                nuke: Arc::new(AtomicBool::new(false)),
                channel,
                thread_handle: None,
            })
    }

    /// Generates the checksum for a given message
    fn checksum(payload: &[u8]) -> u8 {
        payload.iter().fold(0, |acc, x| acc ^ x)
    }

    /// Performs a request with a single response
    fn single_req(&mut self, req: &[u8]) -> Result<Response, RxError> {
        self.channel.write_all(&req)?;
        // response header
        let mut descriptor_bytes = [0u8; 7];

        self.channel.read_exact(&mut descriptor_bytes)?;

        if descriptor_bytes[0..2] != [0xa5, 0x5a] {
            return Err(Corrupted(descriptor_bytes));
        }

        let send_mode = (descriptor_bytes[5] & 0b11000000) >> 6;
        let data_type = descriptor_bytes[6];

        descriptor_bytes[5] = descriptor_bytes[5] ^ (descriptor_bytes[5] & 0b11000000);
        let len = crate::util::read_le_u32(&mut &descriptor_bytes[2..6]);

        let descriptor = ResponseDescriptor {
            len,
            send_mode,
            data_type,
        };

        // data
        let mut data = vec![0u8; descriptor.len as usize];
        self.channel.read_exact(&mut data)?;

        Ok(Response { descriptor, data })
    }

    /// stops the lidar
    pub fn stop(&mut self, reset: bool) {
        self.nuke.store(true, Ordering::Relaxed);
        self.channel
            .write_all(&[0xa5, (if reset { Reset } else { Stop }) as u8])
            .unwrap();
    }

    /// Resets/reboots the lidar
    pub fn reset(&mut self) {
        self.stop(true);
    }

    /// Sets motor speed
    ///
    /// _Unsupported on RPLIDAR S1?_
    ///
    /// TODO check this
    fn set_motor_speed(&mut self, speed: u16) {
        let speed_bytes = speed.to_le_bytes();
        let mut req = [
            0xa5,
            HQMotorSpeedCtrl as u8,
            0x02,
            speed_bytes[0],
            speed_bytes[1],
            0,
        ];
        req[5] = Lidar::checksum(&req);
        self.channel.write_all(&req).unwrap()
    }

    /// Retrieves device information
    pub fn get_info(&mut self) -> SlLidarResponseDeviceInfoT {
        let res = self
            .single_req(&[0xa5, GetDeviceInfo as u8])
            .expect("Could not read device info");
        let data = res.data;

        SlLidarResponseDeviceInfoT {
            model: data[0],
            firmware_version: ((data[2] as u16) << 8) | data[1] as u16,
            hardware_version: data[3],
            serial_number: data[4..20].try_into().unwrap(),
        }
    }

    /// Retrieves the lidar's health
    pub fn get_health(&mut self) -> SlLidarResponseDeviceHealthT {
        let res = self
            .single_req(&[0xa5, GetDeviceHealth as u8])
            .expect("Could not read device health");
        let data = res.data;

        SlLidarResponseDeviceHealthT {
            status: data[0],
            error_code: ((data[2] as u16) << 8) | data[1] as u16,
        }
    }

    pub fn get_health_str(&mut self) -> &'static str {
        ["healthy", "warning", "error"]
            .get(self.get_health().status as usize)
            .unwrap()
    }

    /// Returns the sampling rate of the lidar
    pub fn get_sample_rate(&mut self) -> SlLidarResponseSampleRateT {
        let res = self
            .single_req(&[0xa5, GetSampleRate as u8])
            .expect("Could not read sample rate");
        let data = res.data;

        SlLidarResponseSampleRateT {
            std_sample_duration_us: ((data[1] as u16) << 8) | data[0] as u16,
            express_sample_duration_us: ((data[3] as u16) << 8) | data[2] as u16,
        }
    }

    /// Queries the lidar for specific configuration settings
    pub fn get_lidar_conf(
        &mut self,
        entry: ScanModeConfEntry,
        payload: Option<u16>,
    ) -> SlLidarResponseGetLidarConf {
        let mut req = [0u8; 12];

        req[0] = 0xa5;
        req[1] = GetLidarConf as u8;
        req[3..7].copy_from_slice((entry as u32).to_le_bytes().as_ref());

        match entry {
            Count | Typical => {
                req[2] = 4;
                req[7] = Lidar::checksum(&req[..7]);
            }
            _ => {
                req[2] = 8;
                req[7..9].copy_from_slice(payload.unwrap().to_le_bytes().as_ref());
                req[11] = Lidar::checksum(&req[..11]);
            }
        }

        let res = self
            .single_req(
                &req[..(match entry {
                    Count | Typical => 8,
                    _ => 12,
                })],
            )
            .expect("Could not read lidar conf");
        let data = res.data;

        SlLidarResponseGetLidarConf {
            conf_type: u32::from_le_bytes(data[..4].try_into().unwrap()),
            payload: data[4..].to_owned(),
        }
    }

    /// Requests transmission of laser data from the lidar
    pub fn start_scan(&mut self) -> Result<Receiver<Sample>, serialport::Error> {
        // signal lidar to begin a scan
        self.channel.write_all(&[0xa5, Scan as u8])?;

        let nuke = Arc::clone(&self.nuke);
        let (tx, rx) = mpsc::channel();
        let channel = self
            .channel
            .try_clone()
            .expect("Serial port channels can be cloned.");

        // start reader thread
        self.thread_handle = Some(thread::spawn(move || {
            Self::reader_thread(tx, channel, nuke);
        }));

        Ok(rx)
    }

    /// Thread that receives scan data
    fn reader_thread(tx: Sender<Sample>, mut channel: Box<dyn SerialPort>, nuke: Arc<AtomicBool>) {
        let mut seeking = true;
        let mut descriptor = [0u8; 7];

        channel
            .read_exact(&mut descriptor)
            .expect("missing descriptor");

        if descriptor != [0xa5, 0x5a, 0x05, 0x00, 0x00, 0x40, 0x81] {
            eprintln!("Unable to read lidar stream (malformed descriptor)");
            return;
        }

        // give the lidar time to spin up
        sleep(Duration::from_millis(1000));

        loop {
            const BATCH: usize = S1_BAUD / 500;
            let mut data = [0u8; 5 * BATCH];

            if let Err(err) = channel.read_exact(&mut data) {
                if nuke.load(Ordering::Relaxed) {
                    println!("Scan stopped.");
                    return;
                }
                panic!("Unable to read lidar stream (malformed data): {}", err);
            }

            for i in 0..BATCH {
                let slice = &data[i * 5..i * 5 + 5];

                // checks
                let s = slice[0] & 0b11;
                if s == 0b11 || s == 0b00 || slice[1] & 0b01 != 1 {
                    eprintln!("parity failed: {:x?}", slice);
                    continue;
                }

                let sample = Sample {
                    start: (slice[0] & 1) != 0,
                    intensity: slice[0] >> 2,
                    angle: ((slice[2] as u16) << 1) | (slice[1] as u16 >> 7),
                    distance: (((slice[4] as u16) << 8) | slice[3] as u16) / 4,
                };

                if seeking && !sample.start {
                    continue;
                }

                seeking = false;
                tx.send(sample).unwrap();
            }
        }
    }

    /// Waits for the reader thread to exit.
    pub fn join(&mut self) {
        if let Some(handle) = self.thread_handle.take() {
            handle.join().unwrap();
        }
        self.thread_handle = None;
    }
}
