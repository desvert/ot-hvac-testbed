# Arduino Uno controller

The Arduino Uno is the primary controller for the HVAC testbed.

It owns the control loop and retains full authority over physical outputs, including fan and damper actuation. System state is emitted as structured JSON over serial for host-side logging and analysis.

---

## Build (PlatformIO)

From this directory:

```
pio run
```

## Upload

```
pio run -t upload --upload-port /dev/ttyACM0
```

## Serial monitor

```
pio device monitor -p /dev/ttyACM0 -b 115200
```

## Configuration

Copy the example configuration file:

- `config.example.h` → `config.h`

Then adjust as needed:
- pin assignments
- setpoints and thresholds
- timing constants

The `config.h` file is intentionally ignored by git so wiring and calibration details can remain local.