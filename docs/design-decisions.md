# Design decisions

This document captures a small set of intentional design choices made during development of the HVAC testbed. These decisions favor clarity, predictable behavior, and learning value over performance or feature density.

The goal is not to claim that these choices are universally optimal, but to explain why they are appropriate for this project.

---

## Why an Arduino Uno is the primary controller

The Arduino Uno was selected as the primary controller to keep the control loop simple, deterministic, and locally enforced.

Key reasons:

- The Uno has a single-threaded execution model that makes control flow easy to reason about.
- Actuator control does not depend on an operating system, scheduler, or network stack.
- Failure modes are easier to observe and reproduce during experimentation.
- The platform encourages explicit handling of timing, state, and defaults.

For this testbed, predictability and transparency are prioritized over raw performance or connectivity.

---

## Why the ESP32 is treated as a secondary node

Although the ESP32 is more capable in terms of processing power and connectivity, it is intentionally not used as the primary controller.

Key reasons:

- Network-capable devices introduce additional complexity and failure modes.
- Wireless telemetry is treated as an untrusted input source in an OT context.
- Separating control from connectivity reinforces clear authority boundaries.

The ESP32 is planned as an observability and sensing extension rather than a decision-making component.

---

## Why telemetry is emitted as JSON over serial

Telemetry is emitted as structured JSON over a serial connection to balance simplicity and usefulness.

Key reasons:

- Serial provides a direct, reliable link that does not depend on network configuration.
- JSON is human-readable and easy to inspect during development.
- JSON Lines (`.jsonl`) format supports long-running logs and offline analysis.
- The same telemetry stream can be reused by multiple tools without modification.

This approach keeps data collection decoupled from control logic.

---

## Why the host system is observer-only

The host system is intentionally limited to observability and analysis.

Key reasons:

- Control decisions remain local to the hardware driving the actuators.
- Loss of the host does not affect system behavior.
- Analysis tooling can evolve independently of the control firmware.

This separation mirrors common design patterns in industrial control systems.

---

## Tradeoffs accepted

These design decisions come with known tradeoffs:

- Limited computational resources on the Uno
- Manual configuration and calibration
- Reduced flexibility compared to fully networked controllers

These tradeoffs are accepted in exchange for a system that is easier to understand, reason about, and experiment with safely.

---

## Future considerations

If future phases introduce additional control paths or networked command channels, they will be evaluated against the same principles outlined here:

- Clear authority boundaries
- Explicit failure modes
- Conservative defaults
- Observable behavior
