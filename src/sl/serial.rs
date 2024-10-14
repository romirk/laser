use crate::sl::{Channel, ChannelType};
use serialport::{new, SerialPort};
use std::time::{Duration, Instant};

pub struct SerialPortChannel {
    path: String,
    baud: u32,

    close_pending: bool,
    port: Box<dyn SerialPort>,
}

impl SerialPortChannel {
    pub fn bind(path: String, baud: u32) -> Box<SerialPortChannel> {
        let port = new(&path, baud).timeout(Duration::from_millis(1000)).open().unwrap();
        Box::new(SerialPortChannel {
            path: path.clone(),
            baud,
            close_pending: false,
            port,
        })
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

    fn write(&mut self, data: &[u8]) -> isize {
        self.port.write_all(data).expect("Failed to write serial port!");
        data.len() as isize
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