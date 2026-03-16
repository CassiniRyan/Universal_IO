# G1 IO — Port Reference

## Single-writer rule

**Only `cpp_middleware` ever writes `/lowcmd`.**
All other processes are read-only observers.

---

## Control path (500 Hz via unitree_ros2)

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/lowstate` | `unitree_hg/LowState` | IN | 35 motor states + built-in IMU |
| `/secondary_imu` | `unitree_hg/IMUState` | IN | Torso IMU (quat, gyro, accel, rpy) |
| `/wirelesscontroller` | `unitree_go/WirelessController` | IN | Gamepad axes + button bitmask |
| `/lowcmd` | `unitree_hg/LowCmd` | OUT | PD targets → robot |

---

## Read-only mirrors @ 50 Hz (instinct_onboard exposes subset — we expose all)

### Motor state — all 10 fields from MotorState_ IDL

| Topic | Type | Fields exposed | instinct_onboard? |
|---|---|---|---|
| `/g1/motor_state` | `JointState` | q, dq, tau_est | q, dq only |
| `/g1/motor_ddq` | `Float32MultiArray[29]` | joint acceleration | ✗ missing |
| `/g1/motor_temperature` | `Float32MultiArray[29]` | winding temp (°C) | ✗ missing |
| `/g1/motor_voltage` | `Float32MultiArray[29]` | motor bus voltage (V) | ✗ missing |
| `/g1/motor_status` | `UInt32MultiArray[29]` | fault/error flags | ✗ missing |

### IMU — all fields from IMUState_ IDL

| Topic | Type | Description |
|---|---|---|
| `/g1/imu` | `sensor_msgs/Imu` | quaternion, gyro, accel |
| `/g1/imu_rpy` | `geometry_msgs/Vector3` | roll, pitch, yaw (rad) |

### RC / system

| Topic | Type | Description |
|---|---|---|
| `/g1/rc_cmd` | `geometry_msgs/Twist` | Joystick lx/ly/rx/ry |
| `/g1/tick` | `std_msgs/UInt32` | Hardware timestamp counter |
| `/g1/debug` | `std_msgs/String` | Log messages |

---

## Extra monitoring — unitree_sdk2 DDS thread (optional, separate low-priority thread)

Enable: `colcon build --cmake-args -DUSE_SDK2=ON`  
Only runs if `third_party/unitree_sdk2` is cloned.

### Battery — BmsState_ IDL

| Topic | Type | Description |
|---|---|---|
| `/g1/bms/soc` | `Float32` | State of charge (%) |
| `/g1/bms/current` | `Float32` | Pack current (A, +=charging) |
| `/g1/bms/voltage` | `Float32` | Pack voltage (V) |
| `/g1/bms/temperature` | `Float32MultiArray[12]` | Cell group temperatures (°C) |
| `/g1/bms/cell_voltage` | `Float32MultiArray[40]` | Per-cell voltage (V) |
| `/g1/bms/state` | `String` | JSON summary (soc, soh, cycle) |

### Main board — MainBoardState_ IDL

| Topic | Type | Description |
|---|---|---|
| `/g1/board/fan` | `UInt16MultiArray[6]` | Fan speeds (RPM) |
| `/g1/board/temperature` | `Float32MultiArray[6]` | Board temperatures (°C) |

### Pressure — PressSensorState_ IDL

| Topic | Type | Description |
|---|---|---|
| `/g1/pressure` | `Float32MultiArray[12]` | Foot/hand pressure (N) |

---

## Camera — realsense_node (independent process)

| Topic | Type | Description |
|---|---|---|
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 16UC1 depth |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | XYZ point cloud |

---

## Sim adapter mirrors

| Topic | Type | Description |
|---|---|---|
| `/isaac/joint_states` | `JointState` | G1 state → Isaac Sim |
| `/isaac/imu` | `sensor_msgs/Imu` | IMU → Isaac Sim |
| `/mujoco/joint_states` | `JointState` | G1 state → MuJoCo |
| `/mujoco/imu` | `sensor_msgs/Imu` | IMU → MuJoCo |

---

## Multi-target mode

```bash
ros2 run g1_io cpp_middleware --targets real          # real robot
ros2 run g1_io cpp_middleware --targets real isaac    # real + Isaac
ros2 run g1_io cpp_middleware --targets real mujoco   # real + MuJoCo
ros2 run g1_io cpp_middleware --targets isaac mujoco  # Isaac + MuJoCo
```
