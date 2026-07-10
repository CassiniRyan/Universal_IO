# third_party setup

This folder is kept only for third-party setup notes. The project no longer uses
Pixi; use the system ROS2 Foxy environment plus `setup.sh` from the repo root.

## Layout

```
third_party/
├── setup/
│   └── README.md       ← this file
├── unitree_ros2/       ← clone here (provides unitree_hg / unitree_go msgs)
└── unitree_sdk2/       ← clone here (provides DDS for BMS/board/pressure topics)
```

## Step 1 — Clone dependencies

```bash
# unitree_ros2: provides the ROS2 message packages (unitree_hg, unitree_go)
# and the DDS ↔ ROS2 bridge node
cd unitree_ros2
git clone https://github.com/unitreerobotics/unitree_ros2 .
cd ..

# unitree_sdk2: pre-built C++ library for direct DDS access
# (used only for BmsState, MainBoardState, PressSensorState topics
#  that unitree_ros2 may not bridge)
cd unitree_sdk2
git clone https://github.com/unitreerobotics/unitree_sdk2 .
cd ..
```

## Step 2 — Build unitree_ros2 first

```bash
cd ../unitree_ros2
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Step 3 — Build g1_io

```bash
cd ../../..     # repo root
source setup.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## What each dependency provides

### unitree_ros2
- `unitree_hg` ROS2 message package:
  - `LowState` — all 35 motor states (q, dq, ddq, tau_est, temperature, vol, motorstate)
  - `LowCmd`   — motor commands (q, dq, tau feedforward, kp, kd)
  - `IMUState` — quaternion, gyro, accel, rpy, temperature
- `unitree_go` ROS2 message package:
  - `WirelessController` — joystick axes + buttons
- The bridge node that connects DDS ↔ ROS2 topics

### unitree_sdk2
Used **only** for the three extra monitoring topics that unitree_ros2 may not expose:
- `BmsState_`          → `/g1/bms`    — battery SOC, 40 cell voltages, current, temperatures
- `MainBoardState_`    → `/g1/board`  — fan speeds, board temperatures
- `PressSensorState_`  → `/g1/press`  — 12-channel foot/hand pressure sensors

The main 500 Hz control loop uses **only** unitree_ros2 topics.
The SDK2 subscriber runs in a separate low-priority thread.
