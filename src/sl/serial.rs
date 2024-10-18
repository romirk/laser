use crate::sl::Channel;
use serialport::SerialPort;
use std::io::Write;
use std::time::{Duration, Instant};

pub struct SerialPortChannel {
    close_pending: bool,
    port: Box<dyn SerialPort>,
}

impl SerialPortChannel {
    pub fn bind(path: String, baud: u32) -> Result<Box<SerialPortChannel>, serialport::Error> {
        match serialport::new(&path, baud).timeout(Duration::from_millis(1000)).open() {
            Ok(port) => Ok(Box::new(SerialPortChannel {
                close_pending: false,
                port,
            })),
            Err(err) => Err(err.into())
        }
    }
}
impl Channel for SerialPortChannel {
    fn open(&mut self) -> bool {
        true
    }
    fn close(&mut self) {
        self.close_pending = true;
    }

    fn flush(&mut self) {
        self.port.flush().expect("Failed to flush serial port!");
    }

    fn wait_for_data(&self, size: usize, timeout_ms: u32, actual_ready: &mut usize) -> bool {
        if self.close_pending {
            return false;
        }
        let start = Instant::now();
        let timeout = Duration::from_millis(timeout_ms.into());
        loop {
            *actual_ready = self.port.bytes_to_read().expect("Failed to read serial port!") as usize;
            if *actual_ready >= size {
                return true;
            }
            if start.elapsed() > timeout {
                return false;
            }
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<(), serialport::Error> {
        match self.port.write_all(data) {
            Ok(()) => Ok(()),
            Err(err) => Err(err.into()),
        }
    }

    fn read(&mut self, data: &mut [u8]) -> Result<(), serialport::Error> {
        match self.port.read_exact(data) {
            Ok(()) => Ok(()),
            Err(err) => Err(err.into()),
        }
    }

    fn clear_read_cache() {
        todo!()
    }
}