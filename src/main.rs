extern crate core;

mod sl;
mod util;

use clap::{Parser, Subcommand};
use palette::{Mix, Srgb};
use show_image::{create_window, event, ImageInfo, ImageView};
use sl::lidar::Lidar;
use std::error::Error;
use std::io::Write;
use tqdm::Iter;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    points: Option<usize>,
}

#[show_image::main]
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    // initialize lidar
    let mut lidar = Lidar::init(String::from("COM3"));

    // status information
    let info = lidar.get_info();
    let health = lidar.get_health();

    println!("\nModel {} version {}.{} HW {}", info.model, info.firmware_version >> 8, info.firmware_version & 0xff, info.hardware_version);

    if health.status > 0 {
        eprintln!(" code: {}\nexiting!", health.error_code);
        lidar.reset();
        return Ok(());
    }

    /// number of samples
    let n: usize = args.points.unwrap_or(5000);

    const WIDTH: usize = 1280;
    const HEIGHT: usize = 720;
    let mut pixel_data: Box<[u8]> = vec![0u8; WIDTH * HEIGHT * 3].into_boxed_slice(); // 720p
    let half_diagonal: f32 = (((WIDTH * WIDTH) + (HEIGHT * HEIGHT)) as f32).sqrt();

    let low: Srgb<f32> = Srgb::new(0.50196078431372549019607843137255, 1.0, 0.78823529411764705882352941176471);
    let high: Srgb<f32> = Srgb::new(0.67058823529411764705882352941176, 0.69803921568627450980392156862745, 1.0);

    println!("Starting scan ({} sample{})...", n, if n == 1 { "" } else { "s" });
    let rx = lidar.start_scan();

    // generate pixel data from samples
    println!("Scanning...");
    for (i, sample) in rx.iter().take(n).enumerate().tqdm() {
        let raw_x = (sample.angle as f64).to_radians().cos() * sample.distance as f64;
        let raw_y = (sample.angle as f64).to_radians().sin() * sample.distance as f64;
        let x = ((raw_x / 5f64) as isize + (WIDTH as isize / 2)) as usize;
        let y = ((raw_y / 5f64) as isize + (HEIGHT as isize / 2)) as usize;

        if x < WIDTH && y < HEIGHT {
            let pos = (y * WIDTH + x) * 3;
            let pixel: [f32; 3] = low.mix(high, sample.distance as f32 / half_diagonal).into();
            pixel_data[pos] = (pixel[2] * 255f32) as u8;
            pixel_data[pos + 1] = (pixel[1] * 255f32) as u8;
            pixel_data[pos + 2] = (pixel[0] * 255f32) as u8;
        }
    }

    println!("\nStopping scan...");
    lidar.stop(false);
    lidar.join();

    // display
    let image = ImageView::new(ImageInfo::bgr8(WIDTH as u32, HEIGHT as u32), pixel_data.as_ref());

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
