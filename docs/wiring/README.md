# Wiring notes

This document is intentionally practical. It exists to reflect the current physical wiring of the testbed.

When wiring changes, update this file and add a corresponding photo or diagram. Documentation is treated as part of the system.

---

## Current pin map (Arduino Uno)

The Arduino Uno serves as the primary controller. Pin assignments below reflect the current Phase 2 configuration.

### Inputs

- **Thermistor**  
  A0  
  Voltage divider: 5V → 10kΩ → A0 → NTC → GND

- **BME280 (temperature / humidity / pressure)**  
  I²C  
  SDA: A4  
  SCL: A5  
  Powered from 3.3V

- **T6613 CO₂ sensor**  
  SoftwareSerial  
  RX (Arduino): D10 (from sensor TX)  
  TX (Arduino): D11 (to sensor RX)  
  Baud rate: 19200

- **PIR motion sensor**  
  D2  
  Digital HIGH indicates motion

---

### Outputs

- **Fan control**  
  D5  
  PWM output to MOSFET gate (0–255)

- **Fresh air damper**  
  D9  
  Servo control (0–180 degrees)

- **Occupancy indicator LED**  
  D13

---

## Power and grounding considerations

- All sensor and actuator grounds are common, but current paths matter.
- Servo and fan currents should not return through the same thin jumper grounds used by sensors.
- Unstable readings or erratic behavior should prompt inspection of power distribution and grounding before debugging software.

These issues are common in small testbeds and are treated as part of normal iteration.

---

## Photos and diagrams

Place wiring photos and diagrams in `docs/wiring/images/`.
