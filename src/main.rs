extern crate core;

mod sl;
mod util;

use sl::lidar::Lidar;
use sl::Channel;
use std::io::Write;
use show_image::{ImageInfo, ImageView};

fn main() {
    let mut lidar = Lidar::init(String::from("COM3"));

    let info = lidar.get_info();
    let health = lidar.get_health();
    // let rate = lidar.get_sample_rate();

    println!("\nModel {} version {}.{} HW {}", info.model, info.firmware_version >> 8, info.firmware_version & 0xff, info.hardware_version);
    // println!("Sample rate:\n\tstd: {}us\n\texp: {}us", rate.std_sample_duration_us, rate.express_sample_duration_us);
    println!("Status: {}", ["healthy", "warning", "error"].get(health.status as usize).unwrap());

    if health.status > 0 {
        eprintln!(" code: {}\nexiting!", health.error_code);
        lidar.reset();
        return;
    }

    // let modes = u16::from_le_bytes(lidar.get_lidar_conf(Count, None).payload.try_into().unwrap());
    // let typical = u16::from_le_bytes(lidar.get_lidar_conf(Typical, None).payload.try_into().unwrap());

    // countdown(3);
    println!("Starting scan...");
    lidar.start_scan();

    const N: usize = 5000;
    let mut pixel_data: [u8]= [0u8; 640]; 480];
    // for i in 0..N {
    //     let sample = lidar.get_sample().unwrap();
    //     let x = (sample.angle as f64).cos() * sample.distance as f64;
    //     let y = (sample.angle as f64).sin() * sample.distance as f64;
    //     xs[i] = x;
    //     ys[i] = y;
    // }

    // println!("{:?}", lidar.get_n_samples(5000));

    println!("Stopping scan...");
    lidar.stop(false);
    lidar.join();

    let image = ImageView::new(ImageInfo::rgb8(1920, 1080), &pixel_data);


    const MAX: f64 = 700f64;
    // add histogram to plot
}
