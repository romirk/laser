extern crate core;

mod sl;
mod util;

use sl::cmd::ScanModeConfEntry::{Count, Typical};
use sl::lidar::Lidar;
use sl::Channel;
use std::io::Write;
fn main() {
    let mut lidar = Lidar::init(String::from("COM3"));

    let info = lidar.get_info();
    let health = lidar.get_health();
    let rate = lidar.get_sample_rate();

    println!("\nModel {} version {}.{} HW {}", info.model, info.firmware_version >> 8, info.firmware_version & 0xff, info.hardware_version);
    println!("Sample rate:\n\tstd: {}us\n\texp: {}us", rate.std_sample_duration_us, rate.express_sample_duration_us);
    println!("Status: {}", ["healthy", "warning", "error"].get(health.status as usize).unwrap());

    if health.status > 0 {
        eprintln!(" code: {}\nexiting!", health.error_code);
        lidar.reset();
        return;
    }

    let modes = u16::from_le_bytes(lidar.get_lidar_conf(Count, None).payload.try_into().unwrap());
    let typical = u16::from_le_bytes(lidar.get_lidar_conf(Typical, None).payload.try_into().unwrap());
}
