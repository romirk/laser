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

#[derive(Debug, Clone)]
pub struct Sample {
    pub(crate) start: bool,
    pub(crate) intensity: u8,
    pub(crate) angle: u16,
    pub(crate) distance: u16,
}

pub struct DenseSample {
    pub(crate) start: bool,
    pub(crate) angle: u16,
    pub(crate) cabin: [u16; 40],
}