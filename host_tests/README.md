# Host regression tests

Run from the project root:

```powershell
python -m unittest discover -s host_tests -p "test_*.py" -v
```

These tests verify the protocol/waveform constants and protect the existing
ADC/SD acquisition boundary. The ESP-IDF build remains the authoritative C
compile check; physical timing must still be checked on the target board.

