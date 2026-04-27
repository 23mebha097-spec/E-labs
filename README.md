<h1 align="center">E-Lab</h1>
<p align="center"><b>Programmable 3D Robotic Assembly and Control Environment</b></p>

<p align="center">
  <img alt="Python" src="https://img.shields.io/badge/Python-3.9%2B-3776AB?logo=python&logoColor=white">
  <img alt="PyQt5" src="https://img.shields.io/badge/UI-PyQt5-41CD52?logo=qt&logoColor=white">
  <img alt="PyVista" src="https://img.shields.io/badge/3D-PyVista%20%2F%20VTK-2B3A42">
  <img alt="License" src="https://img.shields.io/badge/License-MIT-blue">
</p>

<p align="center">
  Build custom robots from imported parts, define kinematic chains without URDF,
  simulate motion in real time, and generate ESP32 firmware from your design.
</p>

---

## Why E-Lab?

E-Lab is a desktop robotics lab for rapid prototyping and education:

- Assemble robots from imported meshes (STL/STEP workflows).
- Define parent-child relationships and joints visually.
- Inspect homogeneous transformation matrices live.
- Program robot movement with a simple command script.
- Export firmware logic for ESP32 servo-based hardware.

---

## Core Features

- Link and assembly management
  - Import mechanical links.
  - Set base link and build kinematic hierarchy.
  - Align child links relative to parent links.
- Kinematics engine
  - 4x4 transformation matrix pipeline with NumPy.
  - Forward kinematics propagation through a tree graph.
  - CCD-based inverse kinematics support.
- Interactive 3D simulation
  - PyVista + VTK rendering.
  - Isometric reset and interactive visual controls.
- Programming and experiment panels
  - Scripted command execution for joints.
  - Matrix inspection and debugging tools.
- Firmware generation
  - Generates ESP32 Arduino code for independent joints.
  - Supports smooth non-blocking multi-joint motion.

---

## Tech Stack

- Python
- PyQt5
- PyVista + PyVistaQt (VTK)
- NumPy
- Trimesh
- PySerial

---

## Project Structure

```text
E-lab/
├── main.py
├── core/
│   ├── robot.py
│   └── firmware_gen.py
├── graphics/
│   └── canvas.py
├── ui/
│   ├── main_window.py
│   ├── mixins/
│   ├── panels/
│   └── widgets/
├── firmware/
├── arduino_firmware/
├── assets/
└── tests_*.py / test_*.py
```

---

## Quick Start

### 1. Clone

```bash
git clone https://github.com/ArmanAmreliya/E-Lab.git
cd E-Lab
```

### 2. Create Virtual Environment

```bash
python -m venv .venv
```

Windows (PowerShell):

```powershell
.\.venv\Scripts\Activate.ps1
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Run the App

```bash
python main.py
```

---

## Basic Workflow

1. Open Links panel and import components.
2. Set one link as base.
3. Use Align panel to place child links.
4. Create joints in Joint panel.
5. Test motion and inspect matrices.
6. Write and run simple movement scripts in Code panel.
7. Generate firmware for ESP32 when moving to hardware.

For guided usage details, see [walkthrough.md](walkthrough.md).

---

## Script Example

```text
JOINT Joint_1 45
WAIT 1.0
JOINT Joint_1 -45
```

---

## Firmware Output

Firmware generation produces Arduino-compatible `.ino` code for ESP32 with:

- Joint ID based command parsing (`joint:angle:speed`).
- Smooth incremental servo updates.
- Handshake and runtime serial protocol support.

---

## Development Notes

- Entrypoint: `main.py`
- Main UI shell: `ui/main_window.py`
- Kinematics and IK logic: `core/robot.py`
- ESP32 firmware generator: `core/firmware_gen.py`

---

## Roadmap

- Better asset import and conversion pipeline.
- Enhanced simulation realism.
- Project save/load improvements.
- Expanded robot program command set.
- Additional hardware targets beyond ESP32.

---

## Contributors

Check out our [full list of contributors](CONTRIBUTORS.md).

1. [Arman Amreliya](https://github.com/ArmanAmreliya)
2. [Bhavin](https://github.com/Bhavin)

---

## License

This project is licensed under the [MIT License](LICENSE).
