#!/usr/bin/env python3

import argparse
import os
import sys
from datetime import datetime, timezone

import serial


def utc_iso():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def local_stamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def main():
    ap = argparse.ArgumentParser(description="Log serial JSON lines to a JSONL file.")
    ap.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0, COM3)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    ap.add_argument(
        "--out",
        default=None,
        help="Output path (.jsonl). Default: logs/<timestamp>_hvac.jsonl",
    )
    ap.add_argument(
        "--wrap",
        action="store_true",
        help='Wrap each line as {"ts":"...","data":<line>}',
    )
    args = ap.parse_args()

    out_path = args.out or os.path.join("logs", f"{local_stamp()}_hvac.jsonl")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    print(f"Logging from {args.port} at {args.baud} baud to {out_path}")
    print("Press Ctrl+C to stop.")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"Failed to open serial port: {e}", file=sys.stderr)
        return 2

    try:
        with open(out_path, "a", encoding="utf-8") as f:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if args.wrap:
                    wrapped = f'{{"ts":"{utc_iso()}","data":{line}}}'
                    f.write(wrapped + "\n")
                    f.flush()
                    print(wrapped)
                else:
                    f.write(line + "\n")
                    f.flush()
                    print(line)

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
