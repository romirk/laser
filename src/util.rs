use std::io;
use std::thread::sleep;
use std::time::Duration;
use std::io::Write;

pub fn read_le_u32(input: &mut &[u8]) -> u32 {
    let (int_bytes, rest) = input.split_at(size_of::<u32>());
    *input = rest;
    u32::from_le_bytes(int_bytes.try_into().unwrap())
}

pub fn countdown(secs: i32) {
    print!("\x1b[?25l");
    for i in 0..secs {
        print!("\x1b[2K\r{}", secs - i);
        io::stdout().flush().unwrap();
        sleep(Duration::from_secs(1));
    }
    print!("\x1b[2K\r\x1b[?25h");
}