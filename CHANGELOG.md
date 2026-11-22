# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0](https://github.com/jettify/uf-msp/releases/tag/v0.1.0) - 2026-11-22

### Added

- Implement more packets.
- Address pedantic lints.
- Add macro to automatically implement try_from
- Add macro to simplify commands registration.
- Add read/write bool.
- Consolidate errors make them ergonomic.
- Use heapless for internal vec and string storages, slices on public api.
- Types for variable size payloads.
- Redo module organnization and add codec.
- Implement defmt and add more commands constants.
- Make sure payload size does not mismatch.
- Add trait to decode encode packets.
- Reworked parser inrface, more ergonnomic next()
- Implement decode method for exissting packets
- Implement first few owned packets.
- Polish public API and improve redability.
- Add v1 and v2 over v1 support
- Faster recovery on desync.
- Drop MspPacket struct.
- Setup github CI.
- Add serial example
- Add MspPacket access methods.
- MSP frame slicer.
- Initial commit.

### Fixed

- Upgrade ci commands and install libudev

### Other

- Add plot to readme.
- Setup release pipeline.
- Add examples and fix readme.
- Add license.
- Implement more packet types.
- Add const to several func, drop deref and other small cleanup.
- Rewrok inspect_frame for redundant checks.
- Reformat with nightly formatter
- Do not expose private constatns
- Improve API naming
- Harden parser implementation.
- Make a list of MSP packets.
- Helper methos for more redable parser.
- More idiomatic inspect function.
- Use name constans and improved version of crc over slice.
