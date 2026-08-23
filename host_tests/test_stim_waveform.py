import re
import unittest
from pathlib import Path

try:
    from .test_stim_protocol import GOLDEN_FRAMES
except ImportError:
    from test_stim_protocol import GOLDEN_FRAMES


ROOT = Path(__file__).resolve().parents[1]


def compile_slot(frame):
    result = []
    for byte in frame:
        for bit in range(7, -1, -1):
            result.append((byte >> bit) & 1)  # D0=MOSI, D1=CSb=0
    result.extend([0x03] * 61)  # CSb=1, MOSI=1
    return result


class StimWaveformTests(unittest.TestCase):
    def test_each_slot_has_40_active_and_61_idle_samples(self):
        for frame in GOLDEN_FRAMES:
            slot = compile_slot(frame)
            self.assertEqual(len(slot), 101)
            self.assertTrue(all((sample & 0x02) == 0 for sample in slot[:40]))
            self.assertEqual(slot[40:], [0x03] * 61)

    def test_serial_bits_are_msb_first(self):
        slot = compile_slot(GOLDEN_FRAMES[0])
        self.assertEqual([x & 1 for x in slot[:8]], [0, 1, 0, 0, 0, 0, 0, 0])

    def test_builder_defines_exact_static_buffer_lengths(self):
        header = (ROOT / "main" / "stim_waveform_builder.h").read_text(encoding="utf-8")
        self.assertRegex(header, r"#define\s+STIM_SLOT_SAMPLES\s+101U")
        self.assertRegex(header, r"#define\s+STIM_ACTIVE_SAMPLES\s+40U")
        self.assertRegex(header, r"#define\s+STIM_IDLE_SAMPLES\s+61U")
        self.assertRegex(header, r"#define\s+STIM_START_SEQUENCE_SAMPLES\s+1010U")

    def test_builder_uses_only_d0_and_d1(self):
        source = "\n".join(
            (ROOT / "main" / name).read_text(encoding="utf-8")
            for name in ("stim_waveform_builder.h", "stim_waveform_builder.c")
        )
        self.assertIn("STIM_SAMPLE_MOSI", source)
        self.assertIn("STIM_SAMPLE_CSB", source)
        self.assertNotRegex(source, r"0x0[4-9A-Fa-f]|0x[1-9A-Fa-f]")


if __name__ == "__main__":
    unittest.main()
