use crate::sl::lidar::Lidar;
use sl::Channel;
use std::io::Write;

mod sl;

fn read_le_u32(input: &mut &[u8]) -> u32 {
    let (int_bytes, rest) = input.split_at(size_of::<u32>());
    *input = rest;
    u32::from_le_bytes(int_bytes.try_into().unwrap())
}
fn main() {
    const PORT: &str = "COM3";
    let mut lidar = Lidar::init(PORT.parse().unwrap());
    let info = lidar.get_info();
    let health = lidar.get_health();

    println!("\nModel {} version {}.{} HW {}", info.model, info.firmware_version >> 8, info.firmware_version & 0xff, info.hardware_version);
    println!("Status: {}", ["good", "warning", "error"].get(health.status as usize).unwrap());
}
