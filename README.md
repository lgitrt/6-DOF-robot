# 6-DOF Robotic Arm

A fully 3D-printed six-axis robotic arm inspired by the ABB IRB 1100, built from scratch around
stepper-motor actuation, numerical inverse kinematics, a custom operator GUI, and ROS 2 / micro-ROS
integration.

![6-DOF Robotic Arm](docs/images/arm-overview.jpg)

## Overview

This repository documents the full software stack behind the arm, from the low-level Arduino
firmware up to the operator GUI and ROS teleoperation. Each part of the stack lives in its own
top-level folder:

| Folder | What it is |
|---|---|
| [`python_gui/`](python_gui/) | Desktop operator GUI (PyQt5) — serial link to the Arduino, numerical inverse kinematics, live 3D visualization, and motion record/replay |
| [`firmware/`](firmware/) | Arduino Mega 2560 sketches for stepper motion and encoder feedback |
| [`ros2/`](ros2/) | **Current** ROS 2 keyboard teleoperation node |
| [`ros1_legacy/`](ros1_legacy/) | Legacy ROS 1 teleop prototype, superseded by `ros2/` |
| [`docs/`](docs/) | Inverse kinematics derivation/notebook and README images |

## Features

- **Joint-space & task-space control** panel (`python_gui/`) with live sliders, numeric entry, and
  a one-click "set"/"reset" workflow per joint, talking to the Arduino Mega 2560 over serial.
- **Numerical inverse kinematics** via [IKPY](https://github.com/Phylliade/ikpy), resolving
  task-space targets (x, y, z, phi, theta, psi) to the six joint angles, with a live embedded 3D
  plot of the resulting arm pose.
- **ROS 2 keyboard teleoperation node** publishing incremental `sensor_msgs/JointState` commands
  that the micro-ROS firmware on the Arduino consumes directly.
- **Closed-loop control architecture** with AS5600 magnetic encoders feeding position/velocity
  corrections back to the firmware.
- **Teach-in mode**: record a motion by hand (motors disabled) and replay it at a controlled speed.

| | |
|---|---|
| **Started** | Early 2022 |
| **Inspired by** | ABB IRB 1100 |
| **Actuation** | 6x stepper motors |
| **Low-level control** | Arduino Mega 2560 |
| **Inverse kinematics** | Raspberry Pi, numerical (IKPY) + closed-form (DH-based) |
| **Interfaces** | PyQt5 desktop GUI, micro-ROS / ROS 2, keyboard teleop |

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
├── python_gui/                   # Desktop operator GUI (PyQt5 + IKPY) and the robot URDF
├── firmware/                     # Arduino sketches (stepper control, encoders, FOC test)
│   ├── multi_axis_stepper_accelstepper/
│   ├── multi_axis_stepper_flexystepper/
│   ├── axis6_encoder_test/
│   ├── single_axis_encoder_filter_test/
│   └── stepper_foc_test/
├── ros2/                         # Current ROS 2 keyboard teleoperation node
├── ros1_legacy/                  # Legacy ROS 1 task-space teleop prototype (reference only)
├── docs/
│   ├── images/                   # Screenshots, build photos and CAD renders used in this README
│   └── inverse_kinematics/       # DH derivation (PDF), MATLAB scripts, IKPY demo notebook
├── LICENSE
└── README.md
```

> **Note on CAD/3D models:** the Inventor/SolidWorks source files, STL/STEP exports, and raw
> build photos & videos are maintained outside of this repository (they are large, binary, and not
> diff-friendly for git). A curated set of renders and photos is included under `docs/images/`, and
> the full gallery is available on the
> [project page](https://lucaobwegs.com/6-dof-robotic-arm/).

## Running the GUI

Requirements: Python 3.9+, [IKPY](https://github.com/Phylliade/ikpy), PyQt5, pyserial, matplotlib,
numpy.

```bash
cd python_gui
pip install ikpy pyqt5 pyserial matplotlib numpy
python main.py
```

On start-up the app connects to the Arduino Mega over the serial port configured in `main.py`
(`COM7` by default), loads `arm_urdf.urdf` for inverse kinematics, and opens the controller window
shown in the gallery below.

## Running the ROS 2 keyboard teleop node

Requires a sourced ROS 2 installation (e.g. Humble or newer):

```bash
source /opt/ros/humble/setup.bash
python3 ros2/keyboard_teleop.py --step 2.0 --rate 20
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

- **[`ros1_legacy/legacy_task_space_teleop.py`](ros1_legacy/legacy_task_space_teleop.py)** — an
  early ROS 1 task-space teleop script (adapted from the open-source `teleop_twist_keyboard`
  package), superseded by the ROS 2 node in `ros2/`.
- **[`docs/inverse_kinematics/`](docs/inverse_kinematics/)** — the Denavit–Hartenberg derivation
  (`inverse_kinematics_derivation.pdf`, `transformation_matrices_reference.jpg`), the MATLAB
  symbolic/numeric solution (`matlab/`), and an IKPY-based Jupyter notebook demo
  (`inverse_kinematics_demo.ipynb`).

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
