# Threat model

This document focuses on practical risks to system behavior rather than adversaries alone. It considers incorrect inputs, loss of observability, and unintended actuator behavior within the context of a small HVAC testbed.

This threat model operates within the assumptions and non-goals defined in `docs/overview.md`.

The goal is not to eliminate all risk, but to understand where failures can occur and how design choices limit their impact.

---

## Assets

Assets considered in this model include:

- Controller firmware and configuration values
- Sensor inputs (temperature, CO₂, occupancy)
- Telemetry output emitted by the controller
- Physical actuators driven by the system (fan, damper, indicator LED)

---

## Assumptions

- This is a desktop testbed, not a production HVAC controller
- Physical access to components is assumed
- The Arduino Uno remains the sole authority for actuator control
- The host system provides observability only and does not influence control decisions

---

## Threats and failure modes

The following risks are considered realistic within the scope of this project:

- Incorrect sensor inputs due to wiring faults, sensor drift, failed components, or spoofed signals
- Stale or replayed readings being treated as current system state
- Loss of telemetry due to serial disconnects or host failure, reducing visibility into system behavior
- Configuration mistakes, including incorrect pin assignments, thresholds, or timing constants
- Unintended actuator behavior such as servo binding, fan driver faults, or outputs remaining active when they should be idle

These risks are not limited to malicious activity and may arise from normal development and experimentation.

---

## Mitigations and design choices

The following design choices are used to limit the impact of the identified risks:

- Explicit control boundaries: the Arduino Uno owns all actuator control
- Conservative defaults when sensor reads fail or return invalid data
- Telemetry includes both sensor values and commanded outputs to support reconstruction and analysis
- Output self-test at startup to surface wiring and actuator issues early
- Network-connected components, if added, are treated as untrusted input sources and segmented accordingly

---

## Notes on future ESP32 integration

If an ESP32 module is added for wireless telemetry or additional sensing, it should remain an observer.

Any future command or configuration channel should include authentication, rate limiting, and explicit safe modes, and must not bypass local safety logic enforced on the Arduino Uno.
