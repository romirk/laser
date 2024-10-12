use crate::sl::{Channel, ChannelType};
use serialport::{new, SerialPort};
use std::time::{Duration, Instant};

struct SerialPortChannel {
    path: String,
    baud: u32,

    close_pending: bool,
    port: dyn SerialPort,
}

impl SerialPortChannel {
    fn bind(path: String, baud: u32) -> Box<SerialPortChannel> {
        Box::new(SerialPortChannel {
            path: path.clone(),
            baud,
            close_pending: false,
            port: new(&path, baud),
        })
    }
}
impl Channel for SerialPortChannel {
    fn open(&mut self) -> bool {
        match self.port.open() {
            Ok(_) => true,
            Err(_) => false
        }
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

    fn write(data: &[u8], size: usize) -> isize {
        todo!()
    }

    fn read(&mut self, data: &mut [u8]) -> isize {
        self.port.read_exact(data).expect("Failed to read serial port!");
        data.len() as isize
    }

    fn clear_read_cache() {
        todo!()
    }

    fn get_channel_type() -> ChannelType {
        todo!()
    }
}