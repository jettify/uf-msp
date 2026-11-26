use std::any::type_name;
use std::error::Error;
use std::io::ErrorKind;
use std::io::Read;
use std::io::Write;
use std::io::{self};
use std::time::Duration;
use std::time::Instant;

use uf_msp::MspApiVersion;
use uf_msp::MspDirection;
use uf_msp::MspFcVariant;
use uf_msp::MspParser;
use uf_msp::MspPayload;
use uf_msp::MspVersion;
use uf_msp::write_request_to_buffer;

pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

pub struct Client<T> {
    port: T,
    parser: MspParser<1024>,
    timeout: Duration,
}

impl<T: Read + Write> Client<T> {
    pub fn new(port: T, timeout: Duration) -> Self {
        Self {
            port,
            parser: MspParser::new(),
            timeout,
        }
    }

    pub fn identify(&mut self) -> Result<MspApiVersion> {
        let api = self.query()?;
        let variant: MspFcVariant = self.query()?;
        if variant.identifier != *b"BTFL" {
            let id = String::from_utf8_lossy(&variant.identifier);
            return Err(format!("Expected Betaflight (BTFL), got {id:?}").into());
        }
        Ok(api)
    }

    pub fn optional<P: MspPayload>(&mut self) -> Result<Option<P>> {
        match self.query() {
            Ok(res) => Ok(Some(res)),
            Err(e) if e.kind() == ErrorKind::Unsupported => Ok(None),
            Err(e) => Err(e.into()),
        }
    }

    pub fn query<P: MspPayload>(&mut self) -> io::Result<P> {
        let id = P::COMMAND_ID;
        let name = type_name::<P>().rsplit("::").next().unwrap_or("MSP");

        self.parser.reset();
        let mut req = [0; 16];
        let len = write_request_to_buffer(&mut req, MspVersion::V1, 0, id)
            .map_err(|e| io::Error::new(ErrorKind::InvalidInput, e.to_string()))?;
        self.port
            .write_all(&req[..len])
            .map_err(|e| io::Error::new(e.kind(), format!("Could not send {name}: {e}")))?;

        let start = Instant::now();
        let mut buf = [0; 256];
        while start.elapsed() < self.timeout {
            let len = match self.port.read(&mut buf) {
                Ok(0) => {
                    return Err(io::Error::new(
                        ErrorKind::UnexpectedEof,
                        format!("Serial port disconnected while reading {name}"),
                    ));
                }
                Ok(n) => n,
                Err(e) if matches!(e.kind(), ErrorKind::Interrupted | ErrorKind::TimedOut) => {
                    continue;
                }
                Err(e) => {
                    return Err(io::Error::new(
                        e.kind(),
                        format!("Could not read {name}: {e}"),
                    ));
                }
            };

            let mut input = &buf[..len];
            while let Some(frame) = self.parser.next(&mut input) {
                if frame.command_id() != id || frame.version() != MspVersion::V1 {
                    continue;
                }
                match frame.direction() {
                    MspDirection::Response => {
                        return P::from_bytes(frame.payload()).map_err(|e| {
                            io::Error::new(
                                ErrorKind::InvalidData,
                                format!("Invalid {name} response: {e}"),
                            )
                        });
                    }
                    MspDirection::Error => {
                        return Err(io::Error::new(
                            ErrorKind::Unsupported,
                            format!("Controller rejected {name}"),
                        ));
                    }
                    MspDirection::Request => {}
                }
            }
        }
        Err(io::Error::new(
            ErrorKind::TimedOut,
            format!("No reply to {name}"),
        ))
    }
}
