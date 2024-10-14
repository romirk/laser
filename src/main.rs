extern crate core;

use crate::sl::serial::SerialPortChannel;
use crate::sl::Channel;
use std::io::Write;
use std::thread::sleep;
use std::time::Duration;

mod sl;
mod hal;
mod arch;

fn main() {
    let mut channel = SerialPortChannel::bind("COM3".parse().unwrap(), 256000);


    let req = [0xa5u8, 0x50];
    let mut descriptor = [0u8; 7];
    let mut data = [0u8; 20];

    // let mut buf: [u8;1]= [0];
    // while buf[0] == '-' as u8 {
    //     let mut outgoing = format!("{:-<32}", SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap().as_millis());
    //     channel.write((&mut outgoing).as_ref());
    //     channel.read(&mut buf);
    //     println!("{}", &(buf[0] as char));
    //     sleep(Duration::from_millis(100));
    // }
    //
    // for i in 0..31 {
    //     let mut outgoing = format!("{:-<32}", SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap().as_millis());
    //     channel.write((&mut outgoing).as_ref());
    //     channel.read(&mut buf);
    //     sleep(Duration::from_millis(100));
    // }

    sleep(Duration::from_millis(10));

    channel.write(&req);

    channel.read(&mut descriptor);
    channel.read(&mut data);

    println!("{:x?}", &descriptor);
    println!("{:x?}", data);

    let model = data[0];
    let firmware_minor = data[1];
    let firmware_major = data[2];
    let hardware_version = data[3];
    let serial_number: [u8; 16] = data[4..20].try_into().unwrap();

    println!("\nModel {} version {}.{} HW {} #{:x?}", model, firmware_major, firmware_minor, hardware_version, serial_number);

}
