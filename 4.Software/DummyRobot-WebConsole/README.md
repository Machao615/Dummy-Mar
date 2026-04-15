# DummyRobot Web Console

A standalone web operator console for DummyRobot.

This project is intentionally separate from `DummyRobot-SDK`:

- `DummyRobot-SDK` owns reusable robot APIs
- `DummyRobot-WebConsole` owns the browser UI and operator workflows

## Setup

Install the SDK first:

```bash
pip install -e ../DummyRobot-SDK
```

Install this web console:

```bash
pip install -e .
```

## Run

```bash
uvicorn app.server:app --reload
```

Open:

```text
http://127.0.0.1:8000
```

## Current features

- serial port discovery, connect, disconnect
- robot enable, disable, stop, home, reset
- raw serial command input
- single-joint control and send-all joint control
- live joint angle and tool pose polling
- live connection state, last command, last response, last error
- 3D browser-side robot posture view driven by current joint angles

## Notes

- the 3D model is currently a simplified six-axis kinematic skeleton built with Three.js
- `2.Model` contains STL/STEP assets that can be mapped into the viewer later if needed
- richer physical telemetry still depends on firmware exposing more status commands
- this project uses `DummyRobot-SDK` as its backend control layer
