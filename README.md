# ot-hvac-testbed

A miniature HVAC control system built on microcontrollers for learning building automation, embedded development, and OT security. The testbed simulates real HVAC behavior using temperature, humidity, CO₂, and occupancy inputs with fan and damper actuation outputs.

---

## Project Overview

This project recreates key components of a commercial HVAC control loop on the bench:

- **Arduino Uno** as the primary HVAC controller  
- **Thermistor, BME280, and T6613 CO₂ sensor** for environmental inputs  
- **PIR sensor** for occupancy detection  
- **PWM fan control** through a MOSFET  
- **Servo damper actuator** to simulate outside air control  
- **Structured JSON telemetry** for logging and dashboards  
- **Future integration** with a Pi Zero supervisor and ESP32 secondary sensor node  

The goal is to create a hands-on platform for both HVAC controls learning and OT cybersecurity experimentation.

---

## Architecture

```

[ Sensors ] → Arduino Uno → JSON Telemetry → Pi Zero (logging/dashboard)  
↓  
Fan, Damper, LED Outputs

````

Components:

- **Sensors:** Thermistor, BME280, T6613 CO₂, PIR  
- **Outputs:** Fan (PWM), Damper (servo), Occupancy LED  
- **Future Node:** ESP32 secondary sensor module  
- **Supervisor:** Pi Zero 2W for logging, analysis, and UI  

---

## Control Logic Summary

- Temperature (°F) drives fan speed  
- CO₂ levels adjust damper position and minimum ventilation  
- Occupancy includes a **30-second hold timer** after last motion  
- Environmental data is streamed as structured JSON every 5 seconds  

---

## Hardware Used

- Arduino Uno  
- IRLZ44N MOSFET + flyback diode (fan output)  
- MG90S/SG90  servo (damper)  
- BME280 (I²C)  
- T6613 CO₂ module (UART)  
- PIR sensor module  
- 5V CPU fan  
- Breadboard power module (3.3V / 5V rails)

---

## Firmware

The primary controller firmware:

- Reads all sensors  
- Implements configurable setpoints  
- Applies control logic  
- Outputs JSON structured data  
- Drives fan + damper + LED  

📌 See: `firmware/primary_controller.ino`

---

## Telemetry Format (JSON)

Example output:

```json
{
  "sensors": {
    "thermistorF": 74.39,
    "bmeTempF": 75.38,
    "bmeHumidity": 34.22,
    "co2ppm": 842,
    "pir": true,
    "indoorTempF": 75.38
  },
  "outputs": {
    "fanDuty": 160,
    "damper2Angle": 90,
    "occupied": true
  }
}
````

---

## Logging

A Python script is included for capturing serial JSON data to a `.jsonl` file.  
This will feed into the Pi Zero dashboard in a later phase.

📌 See: `tools/phase2_logger.py`

---

## OT Security Learning Goals

This testbed will evolve into a safe environment for experimenting with:

- Networked sensor data integrity
- MQTT message spoofing / replay
- Poorly secured embedded endpoints
- Hardening microcontroller-based OT systems
- Simulated ICS attack/defense scenarios
    

This mirrors real-world vulnerabilities found in building automation systems.

---

## Roadmap

### Phase 1 — ✔ Thermistor + PWM Fan

### Phase 2 — ✔ Primary HVAC Controller

- Multi-sensor integration
- Servo damper control
- JSON telemetry
- Occupancy hold logic
    

### Phase 3 — ⏳ Pi Zero Supervisor

- Serial ingestion
- Logging and graphing
- Basic dashboard
    

### Phase 4 — ⏳ ESP32 Secondary Node

- Wireless sensor inputs
- MQTT integration
    

### Phase 5 — ⏳ OT Security Layer

- Intentional weaknesses
- Diagnostics and hardening
- Attack simulation

---

## Photos and Diagrams

(Add wiring diagrams, breadboard layouts, and finished build photos here.)

---

## License

MIT License 

---

## Contributions

This is a personal learning project, but PRs and suggestions are welcome!
