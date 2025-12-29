# Architecture

This document describes control boundaries, data flow, and component roles within the HVAC testbed.

The primary design goal is to keep actuator control simple, predictable, and locally enforced, while allowing telemetry and analysis tooling to evolve independently.

Rationale for key architectural choices is documented in `docs/design-decisions.md`.

---

## Control boundaries

The Arduino Uno is the sole authority for actuator control.

- All fan and damper commands originate from the Uno
- External systems may observe system state
- External systems do not directly control physical outputs

If a supervising or networked component fails, the Uno continues operating using conservative defaults.

---

## System overview

```
                           ┌──────────────────────────────────────────┐
                           │        Supervisor / Host System          │
                           │        (Pi Zero 2W or Laptop)            │
                           │                                          │
                           │  - Collects telemetry                    │
                           │  - Stores JSONL logs                     │
                           │  - Hosts dashboards and analysis tools   │
                           └──────────────▲───────────────┬───────────┘
                                          │               │
                              Wired       │               │  WiFi / MQTT / HTTP
                              Serial      │               │  Telemetry only
                                          │               │
                          ┌───────────────┘               │
                          │                               │
┌─────────────────────────┴────────┐               ┌──────┴────────────────────────┐
│          Arduino Uno             │               │          ESP32                │
│     (Primary HVAC Controller)    │               │     (Secondary Sensor Node)   │
│                                  │               │                               │
│  INPUTS:                         │               │  INPUTS (planned):            │
│   - Thermistor                   │               │    - Outdoor temperature      │
│   - BME280 (temp / humidity)     │               │    - Daylight / light level   │
│   - T6613 CO2 sensor             │               │                               │
│   - PIR motion sensor            │               │                               │
│                                  │               │  OUTPUTS:                     │
│  OUTPUTS:                        │               │    - Status LED (optional)    │
│   - PWM fan control              │               │                               │
│   - Servo damper control         │               └───────────────────────────────┘
│   - Occupancy indicator LED      │
└──────────────────────────────────┘
```

---

## Component roles

### Arduino Uno (primary controller)

The Arduino Uno is responsible for all real-time control.

Responsibilities:
- Samples primary environmental sensors
- Derives occupancy state from PIR input
- Applies control logic locally
- Drives fan, damper, and indicator outputs
- Emits system state as structured JSON over serial

The Uno is designed to operate safely without any network dependency.

---

### Supervisor / host system

The supervisor provides observability and analysis only.

Responsibilities:
- Receives telemetry from the Uno
- Stores data as JSON Lines (`.jsonl`)
- Performs visualization and offline analysis

The host does not participate in control decisions and can be disconnected without affecting system behavior.

---

### ESP32 secondary node (planned)

The ESP32 is intended to expand observability, not control.

Responsibilities:
- Provides additional sensing not required for core control
- Publishes telemetry wirelessly to the supervisor

Design constraints:
- No direct actuator control
- Treated as an untrusted input source if networked
- Intended to improve observability rather than authority

---

## Data flow

1. Sensors are sampled by the Arduino Uno
2. Control decisions are made locally on the Uno
3. Actuator outputs are applied
4. System state is serialized as JSON
5. Telemetry is consumed and logged by the host system

---

## Failure considerations

- Loss of host or network connectivity does not affect control behavior
- Sensor read failures result in conservative output defaults
- Output self-test at startup helps identify wiring and actuator issues early
