#!/usr/bin/env python3

import serial
from datetime import datetime

# Adjust this port:
#   - Linux / Pi: '/dev/ttyACM0' or '/dev/ttyUSB0'
#   - Windows: 'COM3', 'COM4', etc.
PORT = '/dev/ttyACM0'
BAUD = 115200
LOGFILE = 'hvac_log.jsonl'

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Logging from {PORT} at {BAUD} baud to {LOGFILE}")

with open(LOGFILE, 'a', encoding='utf-8') as f:
  while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if not line:
      continue

    # Optional: prepend timestamp
    ts = datetime.utcnow().isoformat() + 'Z'
    wrapped = f'{{"ts":"{ts}","data":{line}}}'

    f.write(wrapped + '\n')
    f.flush()
    print(wrapped)
