import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("capture", Path(__file__).parents[1] / "scripts/cpu_load_capture.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)

class Port:
    def __init__(self, lines):
        self.lines = iter(lines)
        self.sent = []
    def write(self, data): self.sent.append(data)
    def readline(self, limit): return next(self.lines)

class CaptureTests(unittest.TestCase):
    def test_valid(self):
        p = Port([b'OK CPUSTAT valid=1 state=OK seq=2 cpu0_busy_pct=82.0\r\n', b'OK END\r\n'])
        self.assertEqual(capture.query(p)['cpu0_busy_pct'], '82.0')
        self.assertEqual(p.sent, [b'CPUSTAT\r\n'])
    def test_invalid_not_zero(self):
        s = capture.parse_snapshot('OK CPUSTAT valid=0 state=STALE cpu0_busy_pct=NA')
        self.assertEqual(s['cpu0_busy_pct'], 'NA')
    def test_old_firmware(self):
        with self.assertRaises(RuntimeError): capture.query(Port([b'ERR COMMAND\r\n']))

if __name__ == '__main__': unittest.main()
