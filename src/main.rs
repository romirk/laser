mod sl;

use sl::lidar::Lidar;
use sl::Channel;
use std::io::Write;

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
    let rate = lidar.get_sample_rate();

    println!("\nModel {} version {}.{} HW {}", info.model, info.firmware_version >> 8, info.firmware_version & 0xff, info.hardware_version);
    println!("Status: {}", ["healthy", "warning", "error"].get(health.status as usize).unwrap());
    println!("Sample rate:\n\tstd: {}us\n\texp: {}us", rate.std_sample_duration_us, rate.express_sample_duration_us);
}
