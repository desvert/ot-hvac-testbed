# ot-hvac-testbed

A small HVAC testbed built to explore control logic, sensor integration, and telemetry from an operational technology perspective.

An Arduino Uno acts as the primary controller and retains full authority over physical outputs. System state is emitted as structured JSON over serial and logged on a host system for analysis. The project is intentionally documented as it evolves, with an emphasis on clear control boundaries and safe default behavior.

---

## High-level design

The system is organized around explicit control boundaries.

**Primary controller (Arduino Uno)**  
- Owns the control loop and all actuator outputs  
- Operates independently of any networked components  
- Defaults to conservative behavior when sensor data is unavailable  

**Supervisor / host system (Linux, Raspberry Pi Zero 2W)**  
- Collects telemetry and stores it for analysis  
- Hosts dashboards and visualization tooling  
- Does not participate in control decisions  

**Future expansion (ESP32 secondary node)**  
- Provides additional sensing and optional network telemetry  
- Augments observability only  
- Does not have authority over actuators  

The Arduino Uno remains the final authority for physical outputs.

---

## Architecture (summary)

Control authority and data flow are intentionally separated.

```

┌─────────────────────────────┐
│  Supervisor / Host System   │
│     (Pi Zero / Laptop)      │
│                             │
│  - Logs telemetry           │
│  - Dashboards / analysis    │
└────────────▲────────────────┘
Serial / USB
┌────────────┴────────────────┐
│        Arduino Uno           │
│  (Primary HVAC Controller)   │
│                              │
│ Sensors → Control → Outputs  │
│                              │
│  Fan / Damper / LED          │
└────────────┬────────────────┘
(future)
Telemetry only
┌────────────┴────────────────┐
│           ESP32              │
│    (Secondary Sensor Node)   │
│                              │
│   Additional sensing         │
└─────────────────────────────┘

```

External systems observe system state but do not participate in control decisions.

A detailed breakdown is available in `docs/architecture.md`.

---

## Hardware used

- Arduino Uno  
- BME280 (I²C)  
- 10k NTC thermistor (analog divider)  
- T6613 CO₂ sensor (UART)  
- PIR motion sensor  
- IRLZ44N MOSFET with flyback diode (fan output)  
- SG90 / MG90S servo (damper)  
- 5V DC fan  
- Breadboard power module (3.3V / 5V rails)  

---

## Control logic summary

- Temperature (°F) determines fan duty cycle  
- CO₂ concentration adjusts damper position and minimum ventilation  
- Occupancy is derived from PIR motion with a 30-second hold timer  
- When unoccupied, outputs return to idle defaults  
- System state is emitted as structured JSON every 5 seconds  

---

## Repository layout

```

ot-hvac-testbed/
├─ firmware/
│  └─ uno-controller/        # Primary HVAC controller (Arduino Uno)
├─ host/
│  └─ logger/                # Python serial logger (JSONL)
├─ docs/                     # Architecture, wiring, threat model
├─ configs/                  # Example pin maps and setpoints
├─ scripts/                  # Helper scripts
└─ media/                    # Photos and diagrams

```

---

## Firmware

The primary controller firmware is built using PlatformIO.

Build:
```

cd firmware/uno-controller
pio run

```

Upload:
```

pio run -t upload --upload-port /dev/ttyACM0

```

---

## Telemetry and logging

The controller emits one JSON object per line over serial.  
A Python logger is included to capture this data into `.jsonl` files for later analysis.

See `host/logger/` for details.

---

## OT security learning goals

This testbed provides a safe environment for experimenting with:

- Sensor data integrity and trust boundaries  
- Replay and spoofing of telemetry streams  
- Poorly secured embedded endpoints  
- Hardening microcontroller-based OT systems  
- Simulated building automation attack and defense scenarios  

These experiments mirror common weaknesses found in real-world building automation systems.

---

## Roadmap

**Phase 1** – Thermistor and PWM fan control  
**Phase 2** – Multi-sensor Uno controller with JSON telemetry  
**Phase 3** – Supervisor logging and visualization (Pi Zero)  
**Phase 4** – ESP32 secondary sensor node  
**Phase 5** – OT security experiments and hardening  

---

## License

MIT