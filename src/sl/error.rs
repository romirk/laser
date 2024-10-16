use std::fmt;
use std::fmt::Formatter;

#[derive(Debug, Clone)]
pub enum RxError {
    Corrupted([u8; 7]),
    PortError(serialport::Error),
    TimedOut,
}

impl fmt::Display for RxError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            RxError::Corrupted(v) => { write!(f, "CORRUPTED! {:x?}", v) }
            RxError::PortError(err) => { write!(f, "Port error: {}", err) }
            RxError::TimedOut => { write!(f, "Timed out waiting for data") }
        }
    }
}