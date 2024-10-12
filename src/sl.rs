use std::ffi::c_float;

mod cmd;
mod types;

/// LIDAR Scan Mode
struct LidarScanMode {
    /// Mode id
    id: u16,

    /// Time cost for one measurement (in microseconds)
    us_per_sample: c_float,

    /// Max distance in this scan mode (in meters)
    max_distance: c_float,

    /// The answer command code for this scan mode
    ans_type: u8,

    /// The name of scan mode (padding with 0 if less than 64 characters)
    scan_mode: [char; 64],
}

enum LIDARTechnologyType {
    LidarTechnologyUnknown = 0,
    LidarTechnologyTriangulation = 1,
    LidarTechnologyDTOF = 2,
    LidarTechnologyETOF = 3,
    LidarTechnologyFMCW = 4,
}

enum LIDARMajorType {
    LidarMajorTypeUnknown = 0,
    LidarMajorTypeASeries = 1,
    LidarMajorTypeSSeries = 2,
    LidarMajorTypeTSeries = 3,
    LidarMajorTypeMSeries = 4,
    LidarMajorTypeCSeries = 6,
}

enum LIDARInterfaceType {
    LidarInterfaceUART = 0,
    LidarInterfaceEthernet = 1,
    LidarInterfaceUSB = 2,
    LidarInterfaceCanbus = 5,

    LidarInterfaceUnknown = 0xFFFF,
}

enum MotorCtrlSupport
{
    MotorCtrlSupportNone = 0,
    MotorCtrlSupportPwm = 1,
    MotorCtrlSupportRpm = 2,
}

enum ChannelType {
    ChannelTypeSerialPort = 0x0,
    ChannelTypeTCP = 0x1,
    ChannelTypeUDP = 0x2,
}

struct SlamtecLidarTimingDesc {
    sample_duration_us: u32,
    native_baudrate: u32,
    linkage_delay_us: u32,
    native_interface_type: LIDARInterfaceType,
    native_timestamp_support: bool,
}

/// Communication channel
trait Channel {
    // IChannel methods

    /// Open communication channel (return true if succeed)
    fn open() -> bool;

    /// Close communication channel
    fn close();

    /// Flush all written data to remote endpoint
    fn flush();

    /// Wait for some data
    ///
    /// * `size` - bytes to wait
    /// * `timeout_ms` - Wait timeout (in microseconds, -1 for forever)
    /// * `actual_ready` - \[out] actual ready bytes
    fn wait_for_data(size: usize, timeout_ms: u32, actual_ready: &mut usize) -> bool;

    /// Send data to remote endpoint
    fn write(data: &[u8], size: usize) -> isize;

    /// Read data from the channel
    fn read(data: &mut [u8], size: usize) -> isize;

    /// Clear read cache
    fn clear_read_cache();
    fn get_channel_type() -> ChannelType;
}

struct SerialPortChannel {

}