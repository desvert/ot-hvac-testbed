# Overview

This project is a desktop HVAC testbed intended for hands-on exploration of control logic and operational telemetry. It is not a production controller.

The primary goal is to build a system that behaves predictably, fails safely, and exposes sufficient internal state to support analysis and learning. Design decisions favor clarity and observability over optimization.

Documentation is treated as part of the system rather than an afterthought.

---

## Assumptions and non-goals

This project is a learning-focused testbed rather than a production control system. Its scope is intentionally limited to support experimentation and iteration.

### Assumptions

- The system is operated in a controlled, non-production environment.
- Hardware, wiring, and configuration may change frequently as the testbed evolves.
- Sensor readings may be incomplete, inaccurate, or unavailable during development.
- The Arduino Uno serves as the primary controller and retains final authority over physical outputs.
- External systems are used to observe system behavior rather than participate in control decisions.

### Non-goals

- This project does not aim to replicate a certified, safety-rated, or code-compliant HVAC controller.
- It is not intended for deployment in real buildings or safety-critical environments.
- Performance optimization is secondary to clarity, observability, and correctness.
- Security hardening is explored incrementally and may include intentionally weak configurations for learning purposes.
- The testbed does not attempt to model every aspect of a commercial building automation system.
