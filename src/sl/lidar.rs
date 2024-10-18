use crate::sl::cmd::ScanModeConfEntry::*;
use crate::sl::cmd::SlLidarCmd::{GetDeviceHealth, GetDeviceInfo, GetLidarConf, GetSampleRate, HQMotorSpeedCtrl, Reset, Scan, Stop};
use crate::sl::cmd::{ScanModeConfEntry, SlLidarResponseDeviceHealthT, SlLidarResponseDeviceInfoT, SlLidarResponseGetLidarConf, SlLidarResponseSampleRateT};
use crate::sl::error::RxError;
use crate::sl::error::RxError::{Corrupted, PortError};
use crate::sl::lidar::LidarState::{Idle, Scanning};
use crate::sl::serial::SerialPortChannel;
use crate::sl::{Channel, Response, ResponseDescriptor};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{mpsc, Arc, Mutex, MutexGuard};
use std::thread;
use std::thread::sleep;
use std::time::Duration;

const S1_BAUD: u32 = 256000;

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
    state: Arc<Mutex<LidarState>>,
    /// serial connection object
    channel: Arc<Mutex<SerialPortChannel>>,

    /// reader thread handle
    thread_handle: Option<thread::JoinHandle<()>>,
}

/// pre-allocated buffer size
const CAPACITY: usize = 2048;

impl Lidar {
    /// initializes a serial connection to the lidar on the given port.
    pub fn init(port: String) -> Lidar {
        match SerialPortChannel::bind(port, S1_BAUD) {
            Ok(channel) => Lidar {
                state: Arc::new(Mutex::new(Idle)),
                channel: Arc::new(Mutex::from(*channel)),
                thread_handle: None,
            },
            Err(e) => panic!("Unable to bind serial port: {}", e),
        }
    }

    /// Generates the checksum for a given message
    fn checksum(payload: &[u8]) -> u8 {
        payload.iter().fold(0, |acc, x| acc ^ x)
    }

    /// Performs a request with a single response
    fn single_req(&mut self, req: &[u8]) -> Result<Response, RxError> {
        let mut channel = self.channel.lock().unwrap();
        match channel.write(&req) {
            Ok(()) => Lidar::rx(channel),
            Err(e) => Err(PortError(e))
        }
    }

    /// Receives a response from the lidar
    fn rx(mut channel: MutexGuard<SerialPortChannel>) -> Result<Response, RxError> {
        // response header
        let mut descriptor_bytes = [0u8; 7];
        match channel.read(&mut descriptor_bytes) {
            Ok(()) => {}
            Err(e) => return Err(PortError(e))
        }

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
        match channel.read(&mut data) {
            Ok(()) => {}
            Err(e) => return Err(PortError(e))
        }

        Ok(Response {
            descriptor,
            data,
        })
    }

    /// stops the lidar
    pub fn stop(&mut self, reset: bool) {
        match self.channel.lock().unwrap().write(&[0xa5, (if reset { Reset } else { Stop }) as u8]) {
            Ok(()) => {
                *self.state.lock().unwrap() = Idle;
                sleep(Duration::from_millis(100));
            }
            Err(e) => panic!("Unable to stop lidar: {}", e),
        }
    }

    /// Resets/reboots the lidar
    pub fn reset(&mut self) { self.stop(true); }

    /// Sets motor speed
    ///
    /// _Unsupported on RPLIDAR S1?_
    ///
    /// TODO check this
    fn set_motor_speed(&mut self, speed: u16) {
        let speed_bytes = speed.to_le_bytes();
        let mut req = [0xa5, HQMotorSpeedCtrl as u8, 0x02, speed_bytes[0], speed_bytes[1], 0];
        req[5] = Lidar::checksum(&req);
        self.channel.lock().unwrap().write(&req).expect("Set motor speed failed");
    }

    /// Retrieves device information
    pub fn get_info(&mut self) -> SlLidarResponseDeviceInfoT {
        let res = self.single_req(&[0xa5, GetDeviceInfo as u8]).expect("Could not read device info");
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
        let res = self.single_req(&[0xa5, GetDeviceHealth as u8]).expect("Could not read device health");
        let data = res.data;

        SlLidarResponseDeviceHealthT {
            status: data[0],
            error_code: ((data[2] as u16) << 8) | data[1] as u16,
        }
    }

    /// Returns the sampling rate of the lidar
    pub fn get_sample_rate(&mut self) -> SlLidarResponseSampleRateT {
        let res = self.single_req(&[0xa5, GetSampleRate as u8]).expect("Could not read sample rate");
        let data = res.data;

        SlLidarResponseSampleRateT {
            std_sample_duration_us: ((data[1] as u16) << 8) | data[0] as u16,
            express_sample_duration_us: ((data[3] as u16) << 8) | data[2] as u16,
        }
    }

    /// Queries the lidar for specific configuration settings
    pub fn get_lidar_conf(&mut self, entry: ScanModeConfEntry, payload: Option<u16>) -> SlLidarResponseGetLidarConf {
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

        let res = self.single_req(&req[..(match entry {
            Count | Typical => 8,
            _ => 12
        })]).expect("Could not read lidar conf");
        let data = res.data;

        SlLidarResponseGetLidarConf {
            conf_type: u32::from_le_bytes(data[..4].try_into().unwrap()),
            payload: data[4..].to_owned(),
        }
    }


    /// Requests transmission of laser data from the lidar
    pub fn start_scan(&mut self) -> Receiver<Sample> {
        // signal lidar to begin a scan
        let channel_arc = self.channel.clone();

        match (|| { return self.channel.lock().unwrap().write(&[0xa5, Scan as u8]); })() {
            Ok(()) => {
                sleep(Duration::from_millis(500));

                *self.state.lock().unwrap() = Scanning;

                let state = Arc::clone(&self.state);

                sleep(Duration::from_millis(1000));

                let (tx, rx) = mpsc::channel();

                // start reader thread
                self.thread_handle = Some(thread::spawn(move || {
                    Self::reader_thread(tx, channel_arc, state);
                }));

                rx
            }
            Err(e) => { panic!("{:?}", e) }
        }
    }

    /// Thread that receives scan data
    fn reader_thread(tx: Sender<Sample>, channel_arc: Arc<Mutex<SerialPortChannel>>, state: Arc<Mutex<LidarState>>) {
        let mut seeking = true;
        let mut descriptor = [0u8; 7];
        {
            channel_arc.lock().unwrap().read(&mut descriptor).expect("missing descriptor");
        }

        loop {
            let mode = state.lock().unwrap().clone();

            match mode {
                Scanning => {}
                mode => {
                    println!("Not scanning: {:?}", mode);
                    break;
                }
            }

            const BATCH: usize = 256;
            let mut data = [0u8; 5 * BATCH];

            match channel_arc.try_lock() {
                Err(_) => {
                    sleep(Duration::from_millis(100));
                    continue;
                }
                Ok(mut channel) =>
                    match channel.read(&mut data) {
                        Err(err) => {
                            println!("Read failure: {}", err);
                            continue;
                        }
                        Ok(()) => {}
                    },
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

                if seeking && !sample.start { continue; }

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

