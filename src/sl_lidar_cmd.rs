// Commands
const SL_LIDAR_AUTOBAUD_MAGICBYTE: u8 = 0x41;

enum SlLidarCmd {
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
    GetSamplerate = 0x59,
    HqMotorSpeedCtrl = 0xA8,

    // Commands with payload and have response
    ExpressScan = 0x82,
    HqScan = 0x83,
    GetLidarConf = 0x84,
    SetLidarConf = 0x85,

    // add for A2 to set RPLIDAR motor pwm when using accessory board
    SetMotorPwm = 0xF0,
    GetAccBoardFlag = 0xFF,
}

// Payload
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

const DEFAULT_MOTOR_SPEED: u16 = 0xFFFF;

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
enum SlLidarAnsType {
    Devinfo = 0x04,
    Devhealth = 0x06,

    Measurement = 0x81,
    MeasurementCapsuled = 0x82,
    MeasurementHq = 0x83,
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

