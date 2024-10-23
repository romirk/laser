use crate::laser::cmd::ScanModeConfEntry;
use crate::laser::Lidar;
use std::error::Error;

pub mod live;

pub fn print_modes(mut lidar: &mut Lidar) -> Result<(), Box<dyn Error>> {
    let modes = u16::from_le_bytes(lidar.get_lidar_conf(ScanModeConfEntry::Count, None).payload.try_into().unwrap());
    let typical = u16::from_le_bytes(lidar.get_lidar_conf(ScanModeConfEntry::Typical, None).payload.try_into().unwrap());
    println!("Modes: {}\nTypical: {}\n", modes, typical);
    for i in 0..modes {
        let name = String::from_utf8(lidar.get_lidar_conf(ScanModeConfEntry::Name, Some(i)).payload).unwrap();
        let us_per_sample = u32::from_le_bytes(lidar.get_lidar_conf(ScanModeConfEntry::UsPerSample, Some(i)).payload.try_into().unwrap()) / (1 << 8);
        let max_distance = u32::from_le_bytes(lidar.get_lidar_conf(ScanModeConfEntry::MaxDistance, Some(i)).payload.try_into().unwrap()) / (1 << 8);
        // let ans_type = match u32::from_le_bytes(lidar.get_lidar_conf(MaxDistance, Some(i)).payload.try_into().unwrap()) / (1 << 8) {
        //     0x81 => Measurement,
        //     0x82 => MeasurementCapsuled,
        //     0x83 => MeasurementHQ,
        //     x => panic!("bad measurement type {}", x),
        // };
        println!("Mode {} - {}", i, name);
        println!("{:-^1$}", "", name.len() + 8);
        println!("   us/sample: {}", us_per_sample);
        println!("max distance: {}m", max_distance);
        // println!("    ans type: {:?}", ans_type);
        println!();
    }
    Ok(())
}