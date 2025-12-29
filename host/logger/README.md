# Serial logger

This utility captures telemetry emitted by the HVAC controller over a serial connection and stores it as JSON Lines (`.jsonl`).

The controller already emits one JSON object per line. The logger operates purely as an observer and does not participate in control decisions. Records may be stored as raw controller output or wrapped with a host-side UTC timestamp for later analysis.

---

## Install

From this directory:

```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

Store raw controller JSON:

```
python log_serial.py --port /dev/ttyACM0
```

Store records wrapped with a host-side UTC timestamp:

```
python log_serial.py --port /dev/ttyACM0 --wrap
```

---

## Output format

Logs are written as JSON Lines, one record per line.

By default, output files are created under:

```
logs/<timestamp>_hvac.jsonl
```

Each line represents a single controller sample and can be processed using standard JSON tooling or ingested into analysis pipelines.

