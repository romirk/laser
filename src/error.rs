use std::{fmt, io};
use std::fmt::Formatter;

#[derive(Debug, Clone)]
pub enum RxError {
    Corrupted([u8; 7]),
    PortError(serialport::Error),
    TimedOut,
}

impl From<serialport::Error> for RxError {
    fn from(e: serialport::Error) -> Self {
        RxError::PortError(e)
    }
}

impl From<io::Error> for RxError {
    fn from(e: io::Error) -> Self {
        Self::PortError(e.into())
    }
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