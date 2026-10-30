#![expect(
    clippy::print_stdout,
    clippy::print_stderr,
    clippy::indexing_slicing,
    reason = "CLI example prints to stdout/stderr and slices read buffer"
)]

use std::io::{ErrorKind, Read, Write};
use std::process::ExitCode;
use std::time::{Duration, Instant};
use uf_msp::{CRC_LEN, HEADER_LEN, MspParser, PREAMBLE_LEN, crc8_dvbs2_slice};

pub const MSP_FC_VARIANT: u16 = 2;
pub const MSP_STATUS: u16 = 101;

fn make_msp_v2_request(command_id: u16, payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(HEADER_LEN + payload.len() + CRC_LEN);
    frame.extend_from_slice(b"$X<");
    frame.push(0); // flags
    frame.extend_from_slice(&command_id.to_le_bytes());
    let len = u16::try_from(payload.len()).unwrap_or(0);
    frame.extend_from_slice(&len.to_le_bytes());
    frame.extend_from_slice(payload);
    let crc = crc8_dvbs2_slice(0, &frame[PREAMBLE_LEN..]);
    frame.push(crc);
    frame
}

fn main() -> ExitCode {
    let default_port = "/dev/tty.usbmodem00000000001B1";
    let port_name = std::env::args()
        .nth(1)
        .unwrap_or_else(|| default_port.to_string());
    println!("Using serial port: {port_name}");

    let Ok(mut port) = serialport::new(&port_name, 115_200)
        .timeout(Duration::from_millis(10))
        .open()
    else {
        eprintln!("Failed to open serial port '{port_name}'");
        return ExitCode::FAILURE;
    };

    let mut buf = [0u8; 1024];
    let mut parser = MspParser::<1024>::new();

    // Send an initial request for FC variant
    let init_req = make_msp_v2_request(MSP_FC_VARIANT, &[]);
    if let Err(e) = port.write_all(&init_req) {
        eprintln!("Failed to send initial request: {e}");
    }

    let poll_interval = Duration::from_millis(100);
    let mut last_poll = Instant::now();

    println!("Reading from serial port '{port_name}'...");
    loop {
        if last_poll.elapsed() >= poll_interval {
            let req = make_msp_v2_request(MSP_STATUS, &[]);
            if let Err(e) = port.write_all(&req) {
                eprintln!("Error sending request: {e}");
            }
            last_poll = Instant::now();
        }

        match port.read(buf.as_mut_slice()) {
            Ok(n) => {
                let mut remaining = &buf[..n];
                loop {
                    let frame = match parser.next(&mut remaining) {
                        Ok(Some(frame)) => frame,
                        Ok(None) => break,
                        Err(error) => {
                            eprintln!("Parser error: {error}");
                            return ExitCode::FAILURE;
                        }
                    };
                    println!(
                        "{:?} command_id={} flags=0x{:02x} len={} payload={:02X?}",
                        frame.direction(),
                        frame.command_id(),
                        frame.flags(),
                        frame.payload().len(),
                        frame.payload()
                    );

                    if frame.command_id() == MSP_FC_VARIANT
                        && let Ok(variant) = core::str::from_utf8(frame.payload())
                    {
                        println!("  -> Flight controller variant: {variant}");
                    }
                }
            }
            Err(ref e) if e.kind() == ErrorKind::TimedOut => {
                // Expected when no data has arrived yet
            }
            Err(e) => {
                eprintln!("Error reading from serial port: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
}
