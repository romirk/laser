pub mod cmd;
pub mod error;
pub mod lidar;
#[derive(Debug)]
pub struct ResponseDescriptor {
    pub len: u32,
    pub send_mode: u8,
    pub data_type: u8,
}

#[derive(Debug)]
pub struct Response {
    pub descriptor: ResponseDescriptor,
    pub data: Vec<u8>,
}

/// Communication channel
pub trait Channel {
    // IChannel methods

    /// Open communication channel (return true if succeed)
    fn open(&mut self) -> bool;

    /// Close communication channel
    fn close(&mut self);

    /// Flush all written data to remote endpoint
    fn flush(&mut self);

    /// Wait for some data
    ///
    /// * `size` - bytes to wait
    /// * `timeout_ms` - Wait timeout (in microseconds, -1 for forever)
    /// * `actual_ready` - \[out] actual ready bytes
    fn wait_for_data(&self, size: usize, timeout_ms: u32, actual_ready: &mut usize) -> bool;

    /// Send data to remote endpoint
    fn write(&mut self, data: &[u8]) -> Result<(), serialport::Error>;

    /// Read data from the channel
    fn read(&mut self, data: &mut [u8]) -> Result<(), serialport::Error>;

    /// Clear read cache
    fn clear_read_cache();
}

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
