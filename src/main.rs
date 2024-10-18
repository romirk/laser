extern crate core;

mod sl;
mod util;

use show_image::{create_window, event, ImageInfo, ImageView};
use sl::lidar::Lidar;
use sl::Channel;
use std::cmp::max;
use std::error::Error;
use std::io::Write;

#[show_image::main]
fn main() -> Result<(), Box<dyn Error>> {
    // initialize lidar
    let mut lidar = Lidar::init(String::from("COM3"));

    // status information
    let info = lidar.get_info();
    let health = lidar.get_health();

    println!("\nModel {} version {}.{} HW {}", info.model, info.firmware_version >> 8, info.firmware_version & 0xff, info.hardware_version);
    println!("Status: {}", ["healthy", "warning", "error"].get(health.status as usize).unwrap());

    if health.status > 0 {
        eprintln!(" code: {}\nexiting!", health.error_code);
        lidar.reset();
        return Ok(());
    }

    /// number of samples
    const N: usize = 4096;

    println!("Starting scan...");
    lidar.start_scan();
    let samples = lidar.get_n_samples(N as u32);
    println!("Stopping scan...");
    lidar.stop(false);
    lidar.join();

    // generate pixel data from samples
    const WIDTH: usize = 1280;
    const HEIGHT: usize = 720;
    let mut pixel_data = [0u8; WIDTH * HEIGHT]; // 720p

    for sample in samples {
        let raw_x = (sample.angle as f64).to_radians().cos() * sample.distance as f64;
        let raw_y = (sample.angle as f64).to_radians().sin() * sample.distance as f64;
        let x = ((raw_x / 5f64) as isize + (WIDTH as isize / 2)) as usize;
        let y = ((raw_y / 5f64) as isize + (HEIGHT as isize / 2)) as usize;

        if x < WIDTH && y < HEIGHT {
            let pos = y * WIDTH + x;
            pixel_data[pos] = max(pixel_data[pos] as u16 + sample.intensity as u16, 255) as u8;
        }
    }

    // display
    let image = ImageView::new(ImageInfo::mono8(WIDTH as u32, HEIGHT as u32), &pixel_data);

    let window = create_window("distance/angle histogram", Default::default())?;
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
