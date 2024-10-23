use rangefinder::laser::Lidar;
use std::error::Error;

// #[show_image::main]
fn main() -> Result<(), Box<dyn Error>> {
    // initialize lidar
    let mut lidar = Lidar::init(String::from("COM3")).expect("Lidar should have initialized");

    #[cfg(feature = "examples")] {
        rangefinder::examples::print_modes(&mut lidar)?;
    }

    Ok(())
}

