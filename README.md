# 6-DOF Robotic Arm

A fully 3D-printed six-axis robotic arm inspired by the ABB IRB 1100, built from scratch around
stepper-motor actuation, numerical inverse kinematics, a custom operator GUI, and ROS 2 / micro-ROS
integration.

![6-DOF Robotic Arm](docs/images/arm-overview.jpg)

## Overview

This repository documents the full software stack behind the arm, from the low-level Arduino
firmware up to the Qt/QML operator GUI and ROS 2 teleoperation:

- **Qt / QML operator GUI** (`content/`, `src/`) — the current human-machine interface, running on
  a Raspberry Pi. It streams joint and task-space commands over a serial link to an Arduino Mega
  2560, drives the six stepper joints in real time, and embeds an RViz view for visual feedback.
- **ROS 2 keyboard teleoperation node** ([keyboard_teleop.py](keyboard_teleop.py)) — publishes
  joint-jog commands that the micro-ROS firmware consumes directly.
- **Arduino firmware prototypes** (`firmware/`) — stepper motion control, AS5600 magnetic encoder
  feedback, and an early field-oriented-control (FOC) feasibility test.
- **Legacy PyQt5 desktop GUI** (`python_gui/`) — the original serial-link controller used before the
  Qt/QML GUI, including the robot's URDF and numerical inverse kinematics via IKPY.
- **Inverse kinematics derivation** (`docs/inverse_kinematics/`) — the Denavit–Hartenberg derivation,
  MATLAB symbolic solution, and an IKPY demo notebook.
- **Legacy ROS 1 teleop prototype** (`ros/`) — an earlier task-space keyboard teleop script, kept for
  reference.

| | |
|---|---|
| **Started** | Early 2022 |
| **Inspired by** | ABB IRB 1100 |
| **Actuation** | 6x stepper motors |
| **Low-level control** | Arduino Mega 2560 |
| **Inverse kinematics** | Raspberry Pi, numerical (cyclic coordinate descent) + closed-form (DH-based) |
| **Interfaces** | Custom Qt/QML GUI, micro-ROS / ROS 2, keyboard teleop |
| **GUI framework** | Qt 6 / QML (Qt Design Studio) |

## Features

- **Joint-space & task-space control** panels with live sliders, numeric entry, and a one-click
  "set"/"reset" workflow per joint.
- **Serial communication panel** for connecting to the Arduino Mega 2560 over USB/UART with live
  connection-status indicators.
- **Embedded RViz view** for real-time visualization of the arm's kinematic state.
- **ROS 2 keyboard teleoperation node** publishing incremental `sensor_msgs/JointState` commands,
  sharing the same command interface as the GUI.
- **Numerical & closed-form inverse kinematics** resolving task-space targets to the six joint
  angles (see `docs/inverse_kinematics/`).
- **Closed-loop control architecture** with AS5600 magnetic encoders feeding position/velocity
  corrections back to the firmware.
- **Teach-in mode** for recording and replaying joint-space waypoints.

## Progress log

- ✅ Inverse kinematics implemented
- ✅ micro-ROS integration — the robot can be controlled from ROS 2
- ✅ GUI for the robot
- ✅ Encoders installed on the joints
- ✅ Teach-in method implemented
- ✅ Switched to a closed-loop control architecture
- 🔜 Migrate low-level control from the Arduino Mega to an STM32H7

## Repository layout

```
6-DOF-robot/
├── content/                     # QML screens (Screen01.ui.qml is the main operator screen)
├── imports/Robot_GUI2/          # Shared QML singletons/components (Constants, font loader, ...)
├── src/                         # C++ application entry point (Qt Quick bootstrap)
├── keyboard_teleop.py           # ROS 2 keyboard teleoperation node (current)
├── CMakeLists.txt               # Top-level CMake build (Qt 6 / qt_add_executable)
├── firmware/                    # Arduino sketches (stepper control, encoders, FOC test)
│   ├── multi_axis_stepper_accelstepper/
│   ├── multi_axis_stepper_flexystepper/
│   ├── axis6_encoder_test/
│   ├── single_axis_encoder_filter_test/
│   └── stepper_foc_test/
├── python_gui/                  # Legacy PyQt5 desktop controller + robot URDF
├── ros/                         # Legacy ROS 1 task-space teleop prototype (reference only)
├── docs/
│   ├── images/                  # Screenshots, build photos and CAD renders used in this README
│   └── inverse_kinematics/      # DH derivation (PDF), MATLAB scripts, IKPY demo notebook
└── LICENSE
```

> **Note on CAD/3D models:** the Inventor/SolidWorks source files, STL/STEP exports, and raw
> build photos & videos are maintained outside of this repository (they are large, binary, and not
> diff-friendly for git). A curated set of renders and photos is included under `docs/images/`, and
> the full gallery is available on the
> [project page](https://lucaobwegs.com/6-dof-robotic-arm/).

## Building the GUI

Requirements: Qt 6.2+ (Qt 6.5 recommended), CMake 3.21+, and a C++ compiler toolchain.

```bash
cmake -B build -S .
cmake --build build
```

The resulting `Robot_GUI2App` binary loads `main.qml`, which hosts `Screen01.ui.qml` — the main
operator screen with the serial connection panel, joint-space/task-space controls, and the
embedded RViz frame.

## Running the keyboard teleop node

Requires a sourced ROS 2 installation (e.g. Humble or newer):

```bash
source /opt/ros/humble/setup.bash
python3 keyboard_teleop.py --step 2.0 --rate 20
```

`q/a`, `w/s`, `e/d`, `r/f`, `t/g`, `y/h` jog joints 1–6, `SPACE` resets to the home position, and
`ESC`/`Ctrl+C` exits. Commands are published on `/joint_commands` as `sensor_msgs/JointState`,
the same message the micro-ROS firmware on the Arduino Mega consumes.

## Firmware

Arduino Mega 2560 sketches under [firmware/](firmware/), built with the Arduino IDE / arduino-cli:

| Sketch | Purpose |
|---|---|
| `multi_axis_stepper_accelstepper/` | Drives all six stepper joints (AccelStepper) with AS5600 encoder feedback |
| `multi_axis_stepper_flexystepper/` | Earlier six-axis step/direction control using FlexyStepper's trapezoidal profiles |
| `axis6_encoder_test/` | Standalone AS5600 encoder readout test for axis 6 |
| `single_axis_encoder_filter_test/` | Position/velocity low-pass filter tuning for a single axis |
| `stepper_foc_test/` | Feasibility test for field-oriented control (SimpleFOC) on a stepper joint |

## Legacy components (kept for reference)

- **`python_gui/`** — the original PyQt5 desktop controller (`main.py`, `robot_gui.py/.ui`) used
  before the Qt/QML GUI, together with the arm's URDF (`arm_urdf.urdf`).
- **`ros/legacy_task_space_teleop.py`** — an early ROS 1 task-space teleop script (adapted from the
  open-source `teleop_twist_keyboard` package), superseded by the ROS 2 node above.
- **`docs/inverse_kinematics/`** — the Denavit–Hartenberg derivation (`inverse_kinematics_derivation.pdf`,
  `transformation_matrices_reference.jpg`), the MATLAB symbolic/numeric solution (`matlab/`), and an
  IKPY-based Jupyter notebook demo (`inverse_kinematics_demo.ipynb`).

## Gallery

| | |
|---|---|
| ![Assembled arm](docs/images/build-01.jpg) | ![Gripper detail](docs/images/gripper-closeup.png) |
| ![Build progress](docs/images/build-02.jpg) | ![Operator GUI](docs/images/gui-screenshot.jpg) |
| ![CAD render - full arm](docs/images/render-arm-cad.png) | ![CAD render - gripper](docs/images/render-gripper.png) |

More photos and build videos are available on the
[project page](https://lucaobwegs.com/6-dof-robotic-arm/).

## License

Distributed under the GNU General Public License v3.0 — see [LICENSE](LICENSE) for details.

## Author

**Luca Obwegs** — Control & Robotics Engineer
[lucaobwegs.com](https://lucaobwegs.com) · [GitHub](https://github.com/lgitrt) ·
[LinkedIn](https://www.linkedin.com/in/luca-obwegs-768716223)
