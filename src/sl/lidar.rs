use std::thread::sleep;
use std::time::Duration;
use crate::sl::cmd::ScanModeConfEntry::*;
use crate::sl::cmd::SlLidarCmd::{GetDeviceHealth, GetDeviceInfo, GetLidarConf, GetSampleRate, HQMotorSpeedCtrl, Reset, Stop};
use crate::sl::cmd::{ScanModeConfEntry, SlLidarResponseDeviceHealthT, SlLidarResponseDeviceInfoT, SlLidarResponseGetLidarConf, SlLidarResponseSampleRateT};
use crate::sl::lidar::LidarState::Idle;
use crate::sl::serial::SerialPortChannel;
use crate::sl::Channel;

const S1_BAUD: u32 = 256000;

enum LidarState {
    Idle,
    Processing,
    Scanning,
    ProtectionStop,
}

pub struct Lidar {
    state: LidarState,
    channel: Box<SerialPortChannel>,
}

pub struct ResponseDescriptor {
    pub len: u32,
    pub send_mode: u8,
    pub data_type: u8,
}

pub struct Response {
    pub descriptor: ResponseDescriptor,
    pub data: Vec<u8>,
}


impl Lidar {
    pub fn init(port: String) -> Lidar {
        Lidar {
            state: Idle,
            channel: SerialPortChannel::bind(port, S1_BAUD),
        }
    }

    fn checksum(payload: &[u8]) -> u8 {
        payload.iter().fold(0, |acc, x| acc ^ x)
    }

    fn single_req(&mut self, req: &[u8]) -> Response {
        // println!(">>> {:x?}", req);
        self.channel.write(&req);

        // response header
        let mut descriptor_bytes = [0u8; 7];
        self.channel.read(&mut descriptor_bytes);
        // print!("<<< {:x?}", &descriptor_bytes);

        // TODO use a result instead
        assert_eq!(descriptor_bytes[0..2], [0xa5, 0x5a]);

        let send_mode = descriptor_bytes[5] & 0b11;
        let data_type = descriptor_bytes[6];

        descriptor_bytes[5] >>= 2;
        let len = crate::read_le_u32(&mut &descriptor_bytes[2..6]);

        let descriptor = ResponseDescriptor {
            len,
            send_mode,
            data_type,
        };

        // data
        let mut data = vec![0u8; descriptor.len as usize];
        self.channel.read(&mut data);
        // println!(" {:x?}", &data);

        Response {
            descriptor,
            data,
        }
    }
    pub fn stop(&mut self) {
        self.channel.write(&[0xa5, Stop as u8]);
        sleep(Duration::from_millis(1));
    }

    pub fn reset(&mut self) {
        self.channel.write(&[0xa5, Reset as u8]);
        sleep(Duration::from_millis(2));
    }

    fn set_motor_speed(&mut self, speed: u16) {
        let speed_bytes = speed.to_le_bytes();
        let mut req = [0xa5, HQMotorSpeedCtrl as u8, 0x02, speed_bytes[0], speed_bytes[1], 0];
        req[5] = Lidar::checksum(&req);
        self.channel.write(&req);
    }

    pub fn get_info(&mut self) -> SlLidarResponseDeviceInfoT {
        let res = self.single_req(&[0xa5, GetDeviceInfo as u8]);
        let data = res.data;

        SlLidarResponseDeviceInfoT {
            model: data[0],
            firmware_version: ((data[2] as u16) << 8) | data[1] as u16,
            hardware_version: data[3],
            serial_number: data[4..20].try_into().unwrap(),
        }
    }

    pub fn get_health(&mut self) -> SlLidarResponseDeviceHealthT {
        let res = self.single_req(&[0xa5, GetDeviceHealth as u8]);
        let data = res.data;

        SlLidarResponseDeviceHealthT {
            status: data[0],
            error_code: ((data[2] as u16) << 8) | data[1] as u16,
        }
    }

    pub fn get_sample_rate(&mut self) -> SlLidarResponseSampleRateT {
        let res = self.single_req(&[0xa5, GetSampleRate as u8]);
        let data = res.data;

        SlLidarResponseSampleRateT {
            std_sample_duration_us: ((data[1] as u16) << 8) | data[0] as u16,
            express_sample_duration_us: ((data[3] as u16) << 8) | data[2] as u16,
        }
    }

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
        })]);
        let data = res.data;

        SlLidarResponseGetLidarConf {
            _type: u32::from_le_bytes(data[..4].try_into().unwrap()),
            payload: data[4..].to_owned()
        }
    }

}

