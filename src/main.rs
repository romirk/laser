extern crate core;

use crate::sl::serial::SerialPortChannel;
use crate::sl::Channel;
use std::io::Write;
use std::thread::sleep;
use std::time::Duration;

mod sl;
mod hal;
mod arch;

fn read_le_u32(input: &mut &[u8]) -> u32 {
    let (int_bytes, rest) = input.split_at(std::mem::size_of::<u32>());
    *input = rest;
    u32::from_le_bytes(int_bytes.try_into().unwrap())
}

const S1_BAUD: u32 = 256000;
const PORT: &'static str = "COM3";

fn main() {
    let mut channel = SerialPortChannel::bind(PORT.parse().unwrap(), S1_BAUD);




    // stabilize
    sleep(Duration::from_millis(10));

    // GET_INFO
    let req = [0xa5u8, 0x50];
    channel.write(&req);

    // response header
    let mut descriptor = [0u8; 7];
    channel.read(&mut descriptor);
    assert!(descriptor[0] == 0xa5 && descriptor[1] == 0x5a);
    let send_mode = descriptor[5] & 0b11;
    let data_type = descriptor[6];

    descriptor[5] >>= 2;
    let len = read_le_u32(&mut &descriptor[2..6]) as usize;

    // data
    let mut data = vec![0u8; len];
    channel.read(&mut data);

    println!("{:x?}", &descriptor);
    println!("{:x?}", data);

    println!("len {} send mode {} type {}", len, send_mode, data_type);

    let model = data[0];
    let firmware_minor = data[1];
    let firmware_major = data[2];
    let hardware_version = data[3];
    let serial_number: [u8; 16] = data[4..20].try_into().unwrap();

    println!("\nModel {} version {}.{} HW {} #{:x?}", model, firmware_major, firmware_minor, hardware_version, serial_number);
}
