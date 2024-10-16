use std::fmt;
use std::fmt::Formatter;

#[derive(Debug, Clone)]
pub struct SerialError;

impl fmt::Display for SerialError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "Serial error")
    }
}