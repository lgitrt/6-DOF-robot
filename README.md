# 6-DOF Robotic Arm

A fully 3D-printed six-axis robotic arm inspired by the ABB IRB 1100, built from scratch around
stepper-motor actuation, numerical inverse kinematics, a custom touchscreen operator GUI, and
ROS 2 / micro-ROS integration.

![6-DOF Robotic Arm](docs/images/arm-overview.jpg)

## Overview

This repository contains the **Qt / QML operator GUI** and the **ROS 2 keyboard teleoperation
node** for the 6-DOF robotic arm project. The GUI runs on a Raspberry Pi and is the primary
human-machine interface for the robot: it streams joint and task-space commands over a serial
link to an Arduino Mega 2560, which drives the six stepper joints in real time, and it embeds an
RViz view for visual feedback.

| | |
|---|---|
| **Started** | Early 2022 |
| **Inspired by** | ABB IRB 1100 |
| **Actuation** | 6x stepper motors |
| **Low-level control** | Arduino Mega 2560 |
| **Inverse kinematics** | Raspberry Pi, cyclic coordinate descent |
| **Interfaces** | Custom Qt/QML GUI, micro-ROS / ROS 2, keyboard teleop |
| **GUI framework** | Qt 6 / QML (Qt Design Studio) |

## Features

- **Joint-space & task-space control** panels with live sliders, numeric entry, and a one-click
  "set"/"reset" workflow per joint.
- **Serial communication panel** for connecting to the Arduino Mega 2560 over USB/UART with live
  connection-status indicators.
- **Embedded RViz view** for real-time visualization of the arm's kinematic state.
- **ROS 2 keyboard teleoperation node** ([keyboard_teleop.py](keyboard_teleop.py)) that publishes
  incremental `sensor_msgs/JointState` commands, sharing the same command interface as the GUI.
- **Numerical inverse kinematics** (cyclic coordinate descent) resolving task-space targets to the
  six joint angles.
- **Closed-loop control architecture** with joint encoders feeding position/torque corrections
  back to the firmware.
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
├── content/                 # QML screens (Screen01.ui.qml is the main operator screen)
├── imports/Robot_GUI2/      # Shared QML singletons/components (Constants, font loader, ...)
├── src/                     # C++ application entry point (Qt Quick bootstrap)
├── keyboard_teleop.py       # ROS 2 keyboard teleoperation node
├── CMakeLists.txt           # Top-level CMake build (Qt 6 / qt_add_executable)
└── docs/images/             # Screenshots and build photos used in this README
```

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

## Gallery

| | |
|---|---|
| ![Assembled arm](docs/images/build-01.jpg) | ![Gripper detail](docs/images/gripper-closeup.png) |
| ![Build progress](docs/images/build-02.jpg) | ![Operator GUI](docs/images/gui-screenshot.jpg) |

More photos and build videos are available on the
[project page](https://lucaobwegs.com/6-dof-robotic-arm/).

## License

Distributed under the GNU General Public License v3.0 — see [LICENSE](LICENSE) for details.

## Author

**Luca Obwegs** — Control & Robotics Engineer
[lucaobwegs.com](https://lucaobwegs.com) · [GitHub](https://github.com/lgitrt) ·
[LinkedIn](https://www.linkedin.com/in/luca-obwegs-768716223)
