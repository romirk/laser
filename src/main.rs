extern crate core;

mod sl;
mod util;

use show_image::{create_window, event, ImageInfo, ImageView};
use sl::lidar::Lidar;
use sl::Channel;
use std::error::Error;
use std::io::Write;

#[show_image::main]
fn main() -> Result<(), Box<dyn Error>> {
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
        return Ok(());
    }

    // let modes = u16::from_le_bytes(lidar.get_lidar_conf(Count, None).payload.try_into().unwrap());
    // let typical = u16::from_le_bytes(lidar.get_lidar_conf(Typical, None).payload.try_into().unwrap());

    // countdown(3);

    const N: usize = 10 * 365;
    const WIDTH: usize = 365;
    const HEIGHT: usize = 100;

    println!("Starting scan...");
    lidar.start_scan();
    let samples = lidar.get_n_samples(N as u32);
    println!("Stopping scan...");
    lidar.stop(false);
    lidar.join();

    let mut pixel_data = [0u8; WIDTH * HEIGHT];
    let mut buckets = [0u16; 365];
    let mut hits = [0u16; 365];
    let mut max = 0;
    for sample in samples {
        if sample.distance > max {
            max = sample.distance;
        }

        let angle = sample.angle as usize;
        buckets[angle] = (sample.distance + buckets[angle] * hits[angle]) / (hits[angle] + 1);
        hits[angle] += 1;

        // let x = ((sample.angle as f64).to_radians().cos() * sample.distance as f64) / 1000f64;
        // let y = ((sample.angle as f64).to_radians().sin() * sample.distance as f64) / 1000f64;
    }

    for depth in 0..HEIGHT {
        let height = HEIGHT - 1 - depth;
        let lower_bound = height * max as usize / HEIGHT;
        for i in 0..buckets.len() {
            if buckets[i] as usize > lower_bound {
                pixel_data[depth * WIDTH + i] = 255
            }
        }
    }

    let image = ImageView::new(ImageInfo::mono8(WIDTH as u32, HEIGHT as u32), &pixel_data);
    // Create a window with default options and display the image.
    let window = create_window("image", Default::default())?;
    window.set_image("image-001", image)?;
    for event in window.event_channel()? {
        if let event::WindowEvent::KeyboardInput(event) = event {
            if event.input.key_code == Some(event::VirtualKeyCode::Escape) && event.input.state.is_pressed() {
                break;
            }
        }
    }

    Ok(())
}
