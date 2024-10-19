extern crate core;

mod sl;
mod util;
mod examples;

use clap::Parser;
use palette::Mix;
use sl::lidar::Lidar;
use std::default::Default;
use std::error::Error;
use std::io::Write;
use tqdm::Iter;
use examples::live;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    points: Option<usize>,
}

// #[show_image::main]
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    // initialize lidar
    let lidar = Lidar::init(String::from("COM3"));

    live::live_view(lidar, args.points)
}
