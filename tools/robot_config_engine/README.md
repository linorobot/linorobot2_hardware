# Linorobot2 Robot Configuration Engine

The **Robot Configuration Engine** is an automated hardware rule validation and code generation tool for Linorobot2. It translates high-level robot hardware specifications (JSON or Web UI) into production-ready C++ firmware headers, PlatformIO build environments, and ROS 2 URDF Xacro descriptions while guarding against electrical and microcontroller-specific hazards.

---

## 🌐 Interactive Web UI

The Configuration Engine includes a **100% client-side, zero-dependency interactive Web UI** located in `tools/robot_config_engine/web/`. It can be opened directly in any modern web browser or hosted statically via GitHub Pages / MkDocs.

- **Live Kinematics HUD**: Real-time calculation of max linear velocity (with 85% PID headroom), angular velocity, and ticks per meter.
- **Hardware Safety Inspector**: Instant visual warnings for duplicate pin collisions, ESP32 input-only GPIOs, boot strapping pins, and RP2040 ADC bounds.
- **Smart Auto-Assign**: One-click conflict-free pin allocation for Pico, Pico 2, ESP32, ESP32-S3, and Waveshare General Driver boards.
- **Multi-Artifact Code Generator**: Real-time tabbed preview and one-click download for C++ config headers, PlatformIO snippets, URDF descriptions, wiring charts, and JSON specifications.

### Launching the Web UI Locally
```bash
cd tools/robot_config_engine/web
python3 -m http.server 8000
# Open http://localhost:8000 in your browser
```

---

## 🚀 CLI Features & Usage

### 1. Validate Specification and Generate Code
```bash
python3 tools/robot_config_engine/generate_config.py examples/scout_pico2.json --out-dir ./output/
```

### 2. Preview in Terminal (No files written)
```bash
python3 tools/robot_config_engine/generate_config.py examples/scout_pico2.json
```

### 3. Run Unit Test Suite
```bash
python3 -m unittest tools/robot_config_engine/test_config_engine.py
```

---

## 📁 File Structure

- `web/`: 100% Client-side interactive Web UI (`index.html`, `style.css`, `app.js`).
- `generate_config.py`: CLI entry point.
- `validator.py`: Rule-based hardware and electrical validation engine.
- `generator.py`: Template generator for C++, PlatformIO, and URDF Xacro.
- `schema.json`: JSON schema definition for robot hardware specifications.
- `test_config_engine.py`: Unit test suite covering valid specs and electrical error conditions.
- `examples/`: Sample JSON specifications (`scout_pico2.json`, `mech_esp32.json`, `crawler_esp32s3.json`).
