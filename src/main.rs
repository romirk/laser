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
        return Ok(())
    }

    // let modes = u16::from_le_bytes(lidar.get_lidar_conf(Count, None).payload.try_into().unwrap());
    // let typical = u16::from_le_bytes(lidar.get_lidar_conf(Typical, None).payload.try_into().unwrap());

    // countdown(3);
    println!("Starting scan...");
    lidar.start_scan();

    const N: usize = 5000;
    const WIDTH: usize = 640;
    const HEIGHT: usize = 480;

    let mut pixel_data = [0u8; (WIDTH * HEIGHT)];
    for sample in lidar.get_n_samples(5000) {
        let x = ((sample.angle as f64).cos() * sample.distance as f64) as i16;
        let y = ((sample.angle as f64).sin() * sample.distance as f64) as i16;
        if x.abs() < 320 && y.abs() < 240 {
            pixel_data[WIDTH * (y + 240) as usize + (x + 320) as usize] = 255;
        }
    }

    // println!("{:?}", lidar.get_n_samples(5000));

    println!("Stopping scan...");
    lidar.stop(false);
    lidar.join();

    let image = ImageView::new(ImageInfo::mono8(WIDTH as u32, HEIGHT as u32), &pixel_data);
    // Create a window with default options and display the image.
    let window = create_window("image", Default::default())?;
    window.set_image("image-001", image)?;

    for event in window.event_channel()? {
        if let event::WindowEvent::KeyboardInput(event) = event {
            println!("{:#?}", event);
            if event.input.key_code == Some(event::VirtualKeyCode::Escape) && event.input.state.is_pressed() {
                break;
            }
        }
    }
    Ok(())
}
