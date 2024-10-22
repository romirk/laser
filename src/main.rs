#![allow(unused, dead_code)]
// TODO REMOVE THIS! ^^

mod examples;
mod sl;
mod util;

use crate::sl::cmd::ScanModeConfEntry::{AnsType, Count, MaxDistance, Name, Typical, UsPerSample};
use clap::Parser;
use sl::lidar::Lidar;
use std::error::Error;
use std::thread::sleep;
use std::time::Duration;
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
    let mut lidar = Lidar::init(args.port)?;
    // live::live_view(lidar, args.points)
    examples::print_modes(&mut lidar)
}

