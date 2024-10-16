// Commands
pub const DEFAULT_MOTOR_SPEED: u16 = 0xFFFF;
const SL_LIDAR_AUTOBAUD_MAGICBYTE: u8 = 0x41;

#[repr(u8)]
pub enum SlLidarCmd {
    // Commands without payload and response
    Stop = 0x25,
    Scan = 0x20,
    ForceScan = 0x21,
    Reset = 0x40,

    // Commands with payload but no response
    NewBaudrateConfirm = 0x90,

    // Commands without payload but have response
    GetDeviceInfo = 0x50,
    GetDeviceHealth = 0x52,
    GetSampleRate = 0x59,
    HQMotorSpeedCtrl = 0xA8,

    // Commands with payload and have response
    ExpressScan = 0x82,
    HQScan = 0x83,
    GetLidarConf = 0x84,
    SetLidarConf = 0x85,

    // add for A2 to set RPLIDAR motor pwm when using accessory board
    SetMotorPWM = 0xF0,
    GetAccBoardFlag = 0xFF,
}

// Payload
// TODO enum this
const SL_LIDAR_EXPRESS_SCAN_MODE_NORMAL: u8 = 0;
const SL_LIDAR_EXPRESS_SCAN_MODE_FIXANGLE: u8 = 0;
// for express working flag(extending express scan protocol)
const SL_LIDAR_EXPRESS_SCAN_FLAG_BOOST: u8 = 0x01;
const SL_LIDAR_EXPRESS_SCAN_FLAG_SUNLIGHT_REJECTION: u8 = 0x02;

//for ultra express working flag
const SL_LIDAR_ULTRAEXPRESS_SCAN_FLAG_STD: u8 = 0x01;
const SL_LIDAR_ULTRAEXPRESS_SCAN_FLAG_HIGH_SENSITIVITY: u8 = 0x02;

struct SlLidarPayloadExpressScanT {
    working_mode: u8,
    working_flags: u16,
    param: u16,
}

struct SlLidarPayloadHqScanT {
    flag: u8,
    reserved: [u8; 32],
}

struct SlLidarPayloadGetScanConfT {
    _type: u32,
}

struct SlPayloadSetScanConfT {
    _type: u32,
}

struct SlLidarPayloadMotorPwmT {
    pwm_value: u16,
}

struct SlLidarPayloadAccBoardFlagT {
    reserved: u32,
}

struct SlLidarPayloadHqSpdCtrlT {
    rpm: u16,
}

struct SlLidarPayloadNewBpsConfirmationT {
    flag: u16,
    required_bps: u32,
    param: u16,
}

// Response
#[derive(Debug)]
#[repr(u8)]
pub enum SlLidarAnsType {
    DevInfo = 0x04,
    DevHealth = 0x06,

    Measurement = 0x81,
    MeasurementCapsuled = 0x82,
    MeasurementHQ = 0x83,
    MeasurementCapsuledUltra = 0x84,
    MeasurementDenseCapsuled = 0x85,
    MeasurementUltraDenseCapsuled = 0x86,

    SampleRate = 0x15,
    GetLidarConf = 0x20,
    SetLidarConf = 0x21,

    AccBoardFlag = 0xFF,
}

const SL_LIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK: u8 = 0x1;

struct SlLidarResponseAccBoardFlagT {
    support_flag: u32,
}

enum SlLidarStatus {
    Ok = 0x0,
    Warning = 0x1,
    Error = 0x2,
}

const SL_LIDAR_RESP_MEASUREMENT_SYNCBIT: u8 = 0x01;
const SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT: u8 = 0x02;

const SL_LIDAR_RESP_HQ_FLAG_SYNCBIT: u8 = 0x01;

const SL_LIDAR_RESP_MEASUREMENT_CHECKBIT: u8 = 0x01;
const SL_LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT: u8 = 0x01;

pub(crate) struct SlLidarResponseSampleRateT {
    pub(crate) std_sample_duration_us: u16,
    pub(crate) express_sample_duration_us: u16,
}

struct SlLidarResponseMeasurementNodeT {
    sync_quality: u8,           // syncbit:1;syncbit_inverse:1;quality:6;
    angle_q6_checkbit: u16,     // check_bit:1;angle_q6:15;
    distance_q2: u16,
}

//[distance_sync flags]
const SL_LIDAR_RESP_MEASUREMENT_EXP_ANGLE_MASK: u8 = 0x3;
const SL_LIDAR_RESP_MEASUREMENT_EXP_DISTANCE_MASK: u8 = 0xFC;

struct SlLidarResponseCabinNodesT {
    distance_angle_1: u16, // see [distance_sync flags]
    distance_angle_2: u16, // see [distance_sync flags]
    offset_angles_q3: u8,
}

const SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_1: u8 = 0xA;
const SL_LIDAR_RESP_MEASUREMENT_EXP_SYNC_2: u8 = 0x5;

const SL_LIDAR_RESP_MEASUREMENT_HQ_SYNC: u8 = 0xA5;

const SL_LIDAR_RESP_MEASUREMENT_EXP_SYNCBIT: u16 = 0x1 << 15;

struct SlLidarResponseCapsuleMeasurementNodesT {
    s_checksum_1: u8, // see [s_checksum_1]
    s_checksum_2: u8, // see [s_checksum_1]
    start_angle_sync_q6: u16,
    cabins: [SlLidarResponseCabinNodesT; 16],
}
struct SlLidarResponseDenseCabinNodesT {
    distance: u16,
}

struct SlLidarResponseDenseCapsuleMeasurementNodesT {
    s_checksum_1: u8, // see [s_checksum_1]
    s_checksum_2: u8, // see [s_checksum_1]
    start_angle_sync_q6: u16,
    cabins: [SlLidarResponseDenseCabinNodesT; 40],
}

struct SlLidarResponseUltraDenseCabinNodesT {
    qualityl_distance_scale: [u16; 2],
    qualityh_array: u8,
}

struct SlLidarResponseUltraDenseCapsuleMeasurementNodesT {
    s_checksum_1: u8, // see [s_checksum_1]
    s_checksum_2: u8, // see [s_checksum_1]
    time_stamp: u32,
    dev_status: u16,
    start_angle_sync_q6: u16,
    cabins: [SlLidarResponseUltraDenseCabinNodesT; 32],
}

const SL_LIDAR_RESP_MEASUREMENT_EXP_ULTRA_MAJOR_BITS: u8 = 12;
const SL_LIDAR_RESP_MEASUREMENT_EXP_ULTRA_PREDICT_BITS: u8 = 10;

struct SlLidarResponseUltraCabinNodesT {
    // 31                                              0
    // | predict2 10bit | predict1 10bit | major 12bit |
    combined_x3: u32,
}

struct SlLidarResponseUltraCapsuleMeasurementNodesT {
    s_checksum_1: u8, // see [s_checksum_1]
    s_checksum_2: u8, // see [s_checksum_1]
    start_angle_sync_q6: u16,
    ultra_cabins: [SlLidarResponseUltraCabinNodesT; 32],
}

struct SlLidarResponseMeasurementNodeHQT {
    angle_z_q14: u16,
    dist_mm_q2: u32,
    quality: u8,
    flag: u8,
}

struct SlLidarResponseHqCapsuleMeasurementNodesT {
    sync_byte: u8,
    time_stamp: u64,
    node_hq: [SlLidarResponseMeasurementNodeHQT; 96],
    crc32: u32,
}

const SL_LIDAR_CONF_SCAN_COMMAND_STD: u8 = 0;
const SL_LIDAR_CONF_SCAN_COMMAND_EXPRESS: u8 = 1;
const SL_LIDAR_CONF_SCAN_COMMAND_HQ: u8 = 2;
const SL_LIDAR_CONF_SCAN_COMMAND_BOOST: u8 = 3;
const SL_LIDAR_CONF_SCAN_COMMAND_STABILITY: u8 = 4;
const SL_LIDAR_CONF_SCAN_COMMAND_SENSITIVITY: u8 = 5;

const SL_LIDAR_CONF_ANGLE_RANGE: u8 = 0x00000000;
const SL_LIDAR_CONF_DESIRED_ROT_FREQ: u8 = 0x00000001;
const SL_LIDAR_CONF_SCAN_COMMAND_BITMAP: u8 = 0x00000002;
const SL_LIDAR_CONF_MIN_ROT_FREQ: u8 = 0x00000004;
const SL_LIDAR_CONF_MAX_ROT_FREQ: u8 = 0x00000005;
const SL_LIDAR_CONF_MAX_DISTANCE: u8 = 0x00000060;
const SL_LIDAR_CONF_LIDAR_MAC_ADDR: u32 = 0x00000079;

#[repr(u32)]
#[derive(Clone, Copy)]
pub enum ScanModeConfEntry {
    Count = 0x00000070,
    UsPerSample = 0x00000071,
    MaxDistance = 0x00000074,
    AnsType = 0x00000075,
    Typical = 0x0000007C,
    Name = 0x0000007F
}


const SL_LIDAR_CONF_MODEL_REVISION_ID: u32 = 0x00000080;
const SL_LIDAR_CONF_MODEL_NAME_ALIAS: u32 = 0x00000081;

const SL_LIDAR_CONF_DETECTED_SERIAL_BPS: u32 = 0x000000A1;

const SL_LIDAR_CONF_LIDAR_STATIC_IP_ADDR: u32 = 0x0001CCC0;
const SL_LIDAR_EXPRESS_SCAN_STABILITY_BITMAP: u8 = 4;
const SL_LIDAR_EXPRESS_SCAN_SENSITIVITY_BITMAP: u8 = 5;

pub(crate) struct SlLidarResponseGetLidarConf {
    pub(crate) conf_type: u32,
    pub(crate) payload: Vec<u8>,
}

struct SlLidarResponseSetLidarConf {
    _type: u32,
    result: u32,
}

pub struct SlLidarResponseDeviceInfoT {
    pub(crate) model: u8,
    pub(crate) firmware_version: u16,
    pub(crate) hardware_version: u8,
    pub(crate) serial_number: [u8; 16],
}

pub struct SlLidarResponseDeviceHealthT {
    pub(crate) status: u8,
    pub(crate) error_code: u16,
}

struct SlLidarIpConfT {
    ip_addr: [u8; 4],
    net_mask: [u8; 4],
    gw: [u8; 4],
}

struct SlLidarResponseDeviceMacaddrInfoT {
    macaddr: [u8; 6],
}

struct SlLidarResponseDesiredRotSpeedT {
    rpm: u16,
    pwm_ref: u16,
}

const SL_LIDAR_VARBITSCALE_X2_SRC_BIT: u8 = 9;
const SL_LIDAR_VARBITSCALE_X4_SRC_BIT: u8 = 11;
const SL_LIDAR_VARBITSCALE_X8_SRC_BIT: u8 = 12;
const SL_LIDAR_VARBITSCALE_X16_SRC_BIT: u8 = 14;

const SL_LIDAR_VARBITSCALE_X2_DEST_VAL: u16 = 512;
const SL_LIDAR_VARBITSCALE_X4_DEST_VAL: u16 = 1280;
const SL_LIDAR_VARBITSCALE_X8_DEST_VAL: u16 = 1792;
const SL_LIDAR_VARBITSCALE_X16_DEST_VAL: u16 = 3328;

const fn sl_lidar_varbitscale_get_src_max_val_by_bits(bits: u32) -> u16 {
    (((0x1 << (bits)) - SL_LIDAR_VARBITSCALE_X16_DEST_VAL) << 4) + ((SL_LIDAR_VARBITSCALE_X16_DEST_VAL - SL_LIDAR_VARBITSCALE_X8_DEST_VAL) << 3) + ((SL_LIDAR_VARBITSCALE_X8_DEST_VAL - SL_LIDAR_VARBITSCALE_X4_DEST_VAL) << 2) + ((SL_LIDAR_VARBITSCALE_X4_DEST_VAL - SL_LIDAR_VARBITSCALE_X2_DEST_VAL) << 1) + SL_LIDAR_VARBITSCALE_X2_DEST_VAL - 1
}

// ^#define\s(\w+)\s+(.*)$
// const $1: u8 = $2;
// sl_(u[0-9]+)\s+(\w+);
// $2: $1,