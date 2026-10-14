#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]

mod parser;

pub use parser::{MspDirection, MspFrame, MspPacket, MspParser, MspVersion};
