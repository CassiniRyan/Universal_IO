# g1_io

Hardware-agnostic IO platform for the Unitree G1 (29-DOF).

## One rule

**The C++ middleware is the only process that ever writes `/lowcmd`.**  
Cameras, Foxglove, RViz, rosbag, policy scripts, sim adapters — all subscribe
to read-only mirror topics. The 500 Hz control loop is never at risk.

## Architecture

```
unitree_ros2 bridge  ──DDS──▶  /lowstate
                               /secondary_imu          ┐
                               /wirelesscontroller      │ inputs
                                      │                 ┘
                         ┌────────────▼────────────┐
                         │    cpp_middleware        │  500 Hz
                         │    G1_middleware.cpp     │◀─── comms loop only
                         │    ├─ safety_filter_0   │◀─── joint pos / vel guard
                         │    └─ safety_filter_1   │◀─── NaN / torque guard
                         └────────────┬────────────┘
                                      │
                    ┌─────────────────┼─────────────────────┐
                    │    /lowcmd      │   /lowcmd_isaac      │  /lowcmd_mujoco
                    ▼                 ▼                       ▼
               real robot        Isaac Sim              MuJoCo
           (unitree_ros2)     (isaaclab_adapter)   (mujoco_adapter)

                                      │
                         read-only mirrors @ 50 Hz
                         /g1/motor_state  /g1/imu  /g1/rc_cmd  /g1/debug
                                      │
              ┌───────────────────────┼──────────────────────┐
              ▼                       ▼                       ▼
          Foxglove                  RViz2               policy script
      ws://localhost:8765        joint_states         (ros2_subscriber.py)

                         /camera/depth/*  (realsense_node — independent)
```

## Layout

```
g1_io/
├── core/
│   ├── bridge/
│   │   ├── cpp_middleware/
│   │   │   ├── include/
│   │   │   │   ├── G1_middleware.hpp   # class declaration
│   │   │   │   └── robot_config.hpp    # joint map, limits, constants
│   │   │   └── src/
│   │   │       └── G1_middleware.cpp   # comms loop — zero safety logic
│   │   └── ros2_msg_types/
│   │       ├── topics.hpp              # all topic names (C++)
│   │       └── topics.py               # all topic names (Python)
│   ├── adapters/
│   │   ├── isaaclab/
│   │   │   └── isaaclab_adapter.py    # mirrors /g1/* → /isaac/*
│   │   ├── mujoco/
│   │   │   └── mujoco_adapter.py      # mirrors /g1/* → /mujoco/*
│   │   └── realsense/
│   │       └── src/realsense_node.cpp # depth camera — independent
│   └── utils/
│       ├── cpp/
│       │   ├── include/
│       │   │   ├── safety_filter_0.hpp  # edit limits here
│       │   │   └── safety_filter_1.hpp
│       │   └── src/
│       │       ├── safety_filter_0.cpp  # pos/vel guard
│       │       └── safety_filter_1.cpp  # NaN/torque/gain guard
│       └── python/
│           ├── ros2_subscriber.py      # thin Python IO reader
│           └── state_estimator.py      # projected gravity, RPY, filter
├── third_party/
│   ├── unitree_ros2/   ← clone https://github.com/unitreerobotics/unitree_ros2
│   └── unitree_sdk2/   ← clone if raw DDS needed (optional for ROS2 path)
├── launch/
│   └── foxglove.launch.py
├── pixi_envs/
│   └── pixi.toml       # ROS2 Humble + onnxruntime + pyrealsense2
├── docs/
│   └── io_ports.md     # full topic/port reference
├── CMakeLists.txt
├── package.xml
├── setup.py
└── setup.sh
```

## Quick start

```bash
# 1. clone deps
cd third_party/unitree_ros2
git clone https://github.com/unitreerobotics/unitree_ros2 .
cd ../..

# 2. bootstrap (installs pixi, creates env)
bash setup.sh

# 3. enter env and build
cd pixi_envs && pixi shell
colcon build --symlink-install
source install/setup.bash

# 4. start unitree_ros2 bridge (separate terminal, on robot network)
ros2 launch unitree_ros2_bringup g1.launch.py eth_name:=eth0

# 5. run middleware  (choose your target)
pixi run run_real            # real robot only
pixi run run_real_isaac      # real + Isaac simultaneously
pixi run run_isaac_mujoco    # Isaac + MuJoCo simultaneously

# 6. optional extras (each in its own terminal)
pixi run realsense           # depth camera
pixi run adapter_isaac       # if using Isaac target
pixi run adapter_mujoco      # if using MuJoCo target
pixi run foxglove            # Foxglove at ws://localhost:8765
```

## Safety

Edit `core/utils/cpp/src/safety_filter_0.cpp` to tune joint position and
velocity limits. Edit `safety_filter_1.cpp` to tune torque and gain bounds.
Neither file has any communication code — you can change limits and rebuild
without reviewing the control loop.

L2+B on the wireless controller triggers damping mode instantly from the
joystick callback, completely independent of the safety filter path.

## Multi-target publishing

The middleware creates one `LowCmd` publisher per `--targets` argument and
writes the same command to all of them every cycle at 500 Hz:

```bash
ros2 run g1_io cpp_middleware --targets real isaac
ros2 run g1_io cpp_middleware --targets real mujoco
ros2 run g1_io cpp_middleware --targets isaac mujoco
```
