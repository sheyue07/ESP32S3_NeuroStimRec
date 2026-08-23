import ast
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "main" / "stim_protocol.c"

GOLDEN_FRAMES = [
    [0x40, 0x23, 0x2A, 0x0D, 0x9A],
    [0x80, 0x23, 0x2A, 0x0D, 0xDA],
    [0x00, 0x63, 0x00, 0x06, 0x69],
    [0x00, 0xA3, 0xC3, 0x23, 0x89],
    [0x00, 0xE3, 0x00, 0x64, 0x47],
    [0x01, 0x23, 0x00, 0x14, 0x38],
    [0x01, 0x63, 0x00, 0x05, 0x69],
    [0x01, 0xA3, 0x00, 0x14, 0xB8],
    [0x01, 0xE3, 0xFF, 0xFF, 0xE2],
    [0x20, 0x23, 0xFF, 0xFF, 0x41],
]


def source_frames():
    text = SOURCE.read_text(encoding="utf-8")
    match = re.search(
        r"stim_start_frames\s*\[[^;=]*=\s*\{(?P<body>.*?)\n\};",
        text,
        flags=re.S,
    )
    if not match:
        raise AssertionError("stim_start_frames initializer not found")
    rows = re.findall(r"\{([^{}]+)\}", match.group("body"))
    return [
        [int(token.strip().rstrip("UuLl"), 0) for token in row.split(",")]
        for row in rows
    ]


class StimProtocolTests(unittest.TestCase):
    def test_ten_golden_frames_match_specification(self):
        self.assertEqual(source_frames(), GOLDEN_FRAMES)

    def test_checksum_is_sum_of_first_four_bytes_modulo_256(self):
        for frame in source_frames():
            self.assertEqual(frame[4], sum(frame[:4]) & 0xFF)

    def test_protocol_frame_size_is_five_bytes(self):
        header = (ROOT / "main" / "stim_protocol.h").read_text(encoding="utf-8")
        self.assertRegex(header, r"#define\s+STIM_PROTOCOL_FRAME_BYTES\s+5U")
        self.assertRegex(header, r"#define\s+STIM_PROTOCOL_START_FRAME_COUNT\s+10U")


if __name__ == "__main__":
    unittest.main()

