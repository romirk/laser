#![allow(unused, dead_code)]
// TODO REMOVE THIS! ^^

mod examples;
mod sl;
mod util;

use clap::Parser;
use examples::live;
use sl::lidar::Lidar;
use std::error::Error;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    port: String,
    points: Option<usize>,
}

// #[show_image::main]
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    // initialize lidar
    let lidar = Lidar::init(args.port)?;
    live::live_view(lidar, args.points)
}
