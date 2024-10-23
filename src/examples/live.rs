use crate::laser::Lidar;
use clap::Parser;
use palette::{Mix, Srgb};
use show_image::{create_window, event, run_context, ImageInfo, ImageView, WindowOptions};
use std::error::Error;
use tqdm::Iter;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    port: String,
    points: Option<usize>,
}

/// Displays a live feed of the lidar stream
pub fn live_view() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    let mut lidar = Lidar::init(args.port).expect("Lidar should have initialized");

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

    // window dimensions
    const WIDTH: usize = 1920;
    const HEIGHT: usize = 1080;
    let mut pixel_data: Box<[u8]> = vec![0u8; WIDTH * HEIGHT * 3].into_boxed_slice();

    // color interpolation
    let high: Srgb<f32> = Srgb::from(0xf3ff82).into();
    let low: Srgb<f32> = Srgb::from(0x1e4160).into();

    // read/display loop
    run_context(move || {
        println!("Starting scan ({} sample{})...", n, if n == 1 { "" } else { "s" });
        let rx = lidar.start_scan().expect("Scan should have started");
        let window = create_window("scan", WindowOptions {
            // size: Some([WIDTH as u32 * 4 / 5, HEIGHT as u32 * 4 / 5]),
            fullscreen: false,
            ..WindowOptions::default()
        }).unwrap();

        // generate pixel data from samples
        println!("Scanning...");
        for (i, sample) in rx.iter().take(n).enumerate().tqdm() {
            let raw_x = (sample.angle as f64).to_radians().cos() * sample.distance as f64;
            let raw_y = (sample.angle as f64).to_radians().sin() * sample.distance as f64;
            let x = ((raw_x / 5f64) as isize + (WIDTH as isize / 2)) as usize;
            let y = ((raw_y / 5f64) as isize + (HEIGHT as isize / 2)) as usize;

            if x < WIDTH && y < HEIGHT {
                let pos = (y * WIDTH + x) * 3;
                let pixel: [f32; 3] = low.mix(high, (sample.intensity as f32) / 64.0).into();
                pixel_data[pos] = (pixel[2] * 255f32) as u8;
                pixel_data[pos + 1] = (pixel[1] * 255f32) as u8;
                pixel_data[pos + 2] = (pixel[0] * 255f32) as u8;
            }

            if i % 360 == 0 {
                // display
                let image = ImageView::new(ImageInfo::bgr8(WIDTH as u32, HEIGHT as u32), pixel_data.as_ref());
                window.set_image(format!("image-{}", i), image).unwrap();
            }
        }

        println!("\nStopping scan...");
        lidar.stop(false);
        lidar.join();

        // listen for ESC
        for event in window.event_channel().unwrap() {
            if let event::WindowEvent::KeyboardInput(event) = event {
                if event.input.key_code == Some(event::VirtualKeyCode::Escape) && event.input.state.is_pressed() {
                    break;
                }
            }
        }
    });
}

