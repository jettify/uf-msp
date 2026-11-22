use uf_msp::MspDirection;
use uf_msp::MspEncodeError;
use uf_msp::MspParser;
use uf_msp::MspVersion;
use uf_msp::commands::MSP_BOARD_INFO;
use uf_msp::write_raw_frame_to_buffer;

fn main() -> Result<(), MspEncodeError> {
    let mut bytes = [0; 32];
    let len = write_raw_frame_to_buffer(
        &mut bytes,
        MspVersion::V1,
        MspDirection::Request,
        0,
        MSP_BOARD_INFO,
        &[],
    )?;

    let mut parser = MspParser::<32>::new();
    for mut input in bytes[..len].chunks(3) {
        while let Some(frame) = parser.next(&mut input) {
            println!(
                "cmd={:#04x}, payload={:?}",
                frame.command_id(),
                frame.payload()
            );
        }
    }
    Ok(())
}
