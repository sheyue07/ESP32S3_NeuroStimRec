"""Read cached CPUSTAT only. Close the exporter/serial monitor before use.

No acquisition commands, flashing or intentional DTR/RTS reset. Some USB
drivers can still pulse lines on open: connect BEFORE beginning acquisition.
"""
import argparse
import csv
from datetime import datetime
import time


def parse_snapshot(line):
    if not line.startswith("OK CPUSTAT "):
        raise ValueError("Not a CPUSTAT response")
    return dict(field.split("=", 1) for field in line.split()[2:])


def query(port):
    port.write(b"CPUSTAT\r\n")
    deadline = time.monotonic() + 5
    result = None
    while time.monotonic() < deadline:
        line = port.readline(1024).decode("ascii", errors="replace").strip()
        if line.startswith("ERR"):
            raise RuntimeError(line)
        if line.startswith("OK CPUSTAT "):
            result = parse_snapshot(line)
        if line == "OK END" and result is not None:
            return result
    raise TimeoutError("CPUSTAT timeout: stop polling; check firmware/port (no reset attempted)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    if args.interval < 1:
        parser.error("interval must be >= 1 second")
    import serial
    output = args.output or datetime.now().strftime("cpu_load_%Y%m%d_%H%M%S.csv")
    port = serial.Serial(port=None, baudrate=args.baud, timeout=0.5,
                         write_timeout=2, rtscts=False, dsrdtr=False)
    port.dtr = False
    port.rts = False
    port.port = args.port
    # Exclusive-create preserves any previous experiment log.
    with open(output, "x", newline="", encoding="utf-8-sig") as stream:
        port.open()
        try:
            writer = None
            print("Connected for CPUSTAT only. Ctrl+C stops logging; it does NOT stop ADC.")
            while True:
                row = {"host_time": datetime.now().astimezone().isoformat(), **query(port)}
                if writer is None:
                    writer = csv.DictWriter(stream, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
                stream.flush()
                print(row)
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("Saved partial log:", output)
        finally:
            port.close()


if __name__ == "__main__":
    main()
