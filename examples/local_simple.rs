use uf_msp::MspParser;

fn main() {
    // MSP v1 response to MSP_API_VERSION with payload [0, 1, 49].
    let bytes = b"$M>\x03\x01\x00\x01\x31\x32";
    let mut parser = MspParser::<32>::new();
    let mut input = bytes.as_slice();

    for result in parser.packets(&mut input) {
        match result {
            Ok(packet) => println!("{:?}", packet.message),
            Err(error) => eprintln!("{error}"),
        }
    }
}
