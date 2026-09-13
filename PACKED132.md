# 2026.09.04-r6-packed132

Only the CPU1 recorder packs validated frames, immediately before the eMMC
write cache. DMA, frame synchronization, padding validation, the 7+7 MiB
RAW260 rings, and BLE preview remain unchanged. No payload checksum is added
to the acquisition path. Existing export transport CRC remains unchanged.

Input: `FFFF0000 + 64 * (sample[2] + zero[2])`, 260 bytes.
Stored: `FFFF0000 + 64 * sample[2]`, 132 bytes, original byte order.
Space reduction: 128/260 = 49.2308%. No sample bits are discarded.

## Format and accounting

- Superblock and event sectors remain version 2 (unchanged layouts).
- New segment entries are version 3: PACKED132. Version 2 entries are RAW260.
- Version is covered by the existing metadata checksum. Warm resume preserves
  old entries and validates each using its own frame size, allowing mixed runs.
- Physical/valid byte counts are stored bytes; frame counts use 132 for new
  segments. Raw input and synchronization event bit offsets still use RAW260.
- Recorder append consumes whole RAW260 frames and may split the packed output
  at the 64 KiB write boundary. Checkpoints and final valid length exclude any
  incomplete packed frame. Final sectors may contain non-data tail padding.
- STATUS raw_mib_s is now raw input throughput; end_to_end_mib_s is physical
  eMMC throughput. Approximately 3.58 -> 1.82 MiB/s is expected, not data loss.

## Export and testing

Use exporter v1.7.0. LIST includes format and frame_bytes. Exported PACKED132
files use `_packed132.bin` with a JSON sidecar (also applies to `.bin.part`).
Existing RAW260 analysis scripts cannot directly parse PACKED132. Offline
reconstruction is available in the exporter source:
`python -m emmc_exporter.packed input_packed132.bin output_raw260.bin`.
It restores only validated complete frames; it refuses unknown headers or a
partial trailing frame. For LBA excerpts, calculate the byte offset relative
to the segment before decoding; do not hunt arbitrary sample bytes for headers.

This change does not alter power-session policy: the first capture after a
true power cycle replaces the old run. Export valuable data before testing.
Firmware has not been flashed by the assistant; serial hardware is untouched.

Hardware acceptance: compare CPU0/CPU1 to the RAW260 baseline under identical
preview settings; test short/long capture and stop, events, and exported samples
against a standard recording. Compilation alone does not prove hardware stability.
