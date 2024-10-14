mod sl;

use crate::sl::cmd::ConfEntry::{Count, MaxDistance, Name, Typical, UsPerSample};
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

    let modes = u16::from_le_bytes(lidar.get_lidar_conf(Count, None).payload.try_into().unwrap());
    let typical = u16::from_le_bytes(lidar.get_lidar_conf(Typical, None).payload.try_into().unwrap());

    println!("Modes: {}\nTypical: {}\n", modes, typical);

    for i in 0..modes {

        let us_per_sample = u32::from_le_bytes(lidar.get_lidar_conf(UsPerSample, Some(i)).payload.try_into().unwrap()) / (1 << 8);
        let max_distance = u32::from_le_bytes(lidar.get_lidar_conf(MaxDistance, Some(i)).payload.try_into().unwrap()) / (1 << 8);
        // let ans_type = match u32::from_le_bytes(lidar.get_lidar_conf(MaxDistance, Some(i)).payload.try_into().unwrap()) / (1 << 8) {
        //     0x81 => Measurement,
        //     0x82 => MeasurementCapsuled,
        //     0x83 => MeasurementHQ,
        //     x => panic!("bad measurement type {}", x),
        // };
        let name = String::from_utf8(lidar.get_lidar_conf(Name, Some(i)).payload).unwrap();

        println!("Mode {} - {}", i, name);
        println!("{:-^1$}", "", name.len() + 8);
        println!("   us/sample: {}", us_per_sample);
        println!("max distance: {}m", max_distance);
        // println!("    ans type: {:?}", ans_type);

        println!();
    }
}
