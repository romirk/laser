mod lidar;
pub(crate) mod cmd;
mod protocol;

pub use lidar::Lidar;

// LIDAR Scan Mode
// pub struct LidarScanMode {
//     /// Mode id
//     id: u16,
//
//     /// Time cost for one measurement (in microseconds)
//     us_per_sample: c_float,
//
//     /// Max distance in this scan mode (in meters)
//     max_distance: c_float,
//
//     /// The answer command code for this scan mode
//     ans_type: u8,
//
//     /// The name of scan mode (padding with 0 if less than 64 characters)
//     scan_mode: [char; 64],
// }
