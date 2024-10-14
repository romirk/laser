use crate::sl::cmd::SlLidarResponseDeviceInfoT;
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

    fn single_req(&mut self, req: &[u8]) -> Response {
        self.channel.write(&req);

        // response header
        let mut descriptor_bytes = [0u8; 7];
        self.channel.read(&mut descriptor_bytes);

        // TODO use a result instead
        assert!(descriptor_bytes[0] == 0xa5 && descriptor_bytes[1] == 0x5a);

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
        Response {
            descriptor,
            data,
        }
    }

    pub fn get_info(&mut self) -> SlLidarResponseDeviceInfoT {
        let req = [0xa5u8, 0x50];
        let res = self.single_req(&req);

        let data = res.data;
        let model = data[0];
        let firmware_version: u16 = ((data[2] as u16) << 8) | data[1] as u16;
        println!("Firmware Version: {}.{} ", firmware_version >> 8, firmware_version & 0xff);
        let hardware_version = data[3];
        let serial_number: [u8; 16] = data[4..20].try_into().unwrap();

        SlLidarResponseDeviceInfoT {
            model,
            firmware_version,
            hardware_version,
            serial_number,
        }
    }
}

