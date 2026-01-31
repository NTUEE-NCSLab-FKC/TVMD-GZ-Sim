# TVMD-GZ-Sim Data Sources Documentation

This document explains the data sources for joint angles, thrust vectors, and body wrench in the TVMD-GZ-Sim project.

## Overview

```
PX4 Firmware (Control Allocation)
         |
         v
    ROS Topics
         |
    +----+----+--------------------+
    |         |                    |
    v         v                    v
ActuatorServos  ActuatorMotors  ControlAllocationMetaData
    |              |                    |
    v              v                    v
Joint Angles    Thrust           Body Wrench
```

---

## 1. Joint Angles Data Source

### ROS Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `px4/actuator_servos` | `ActuatorServos` | 8 servo control values |
| `px4/actuator_motors` | `ActuatorMotors` | 4 motor control values |

### Message Structure

**ActuatorServos.msg** (`src/px4_msgs/msg/ActuatorServos.msg`)
```
uint64 timestamp
float32[8] control    # Range: [-1, 1]
```

**Control Index Mapping:**
| Index | Module | Joint Type |
|-------|--------|------------|
| 0 | Module 1 | gimbal_actuator (X-axis) |
| 1 | Module 1 | body_gimbal (Y-axis) |
| 2 | Module 2 | gimbal_actuator (X-axis) |
| 3 | Module 2 | body_gimbal (Y-axis) |
| 4 | Module 3 | gimbal_actuator (X-axis) |
| 5 | Module 3 | body_gimbal (Y-axis) |
| 6 | Module 4 | gimbal_actuator (X-axis) |
| 7 | Module 4 | body_gimbal (Y-axis) |

### Angle Conversion Formula

Defined in `src/tvmd/republisher.py:367-396`:

```python
joint_angle = control_value * (max - min) / 2 + (max + min) / 2
```

### Joint Limits

From `src/tvmd/urdf/Single-Agent-Def.xacro`:

| Joint | Axis | Min | Max | Description |
|-------|------|-----|-----|-------------|
| `body_gimbal_joint` | Y | -0.7854 rad (-45 deg) | 0.7854 rad (45 deg) | Gimbal tilt |
| `gimbal_actuator_joint` | X | -0.349 rad (-20 deg) | 0.349 rad (20 deg) | Actuator tilt |
| `actuator_prop1_joint` | Z | -inf | inf | Propeller rotation (continuous) |

---

## 2. Thrust Vector Data Source

### Data Flow

```
ActuatorMotors (control[0-3])
         |
         v
   Motor Angular Velocity
   omega = control * maxRotVelocity
         |
         v
   Gazebo MulticopterMotorModel Plugin
         |
         v
   Thrust Calculation
   thrust = motorConstant * omega^2
```

### Motor Parameters

From `src/tvmd/urdf/tvmd.gazebo:38-57`:

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `motorConstant` | 2e-05 | N/(rad/s)^2 | Thrust coefficient |
| `momentConstant` | 0.06 | - | Torque coefficient |
| `maxRotVelocity` | 1500 | rad/s | Maximum angular velocity |
| `timeConstantUp` | 0.0125 | s | Spin-up time constant |
| `timeConstantDown` | 0.025 | s | Spin-down time constant |

### Thrust Direction

The thrust direction is determined by the servo angles:

```python
# Rotation matrices
R_y = rotation_about_Y(eta_y)  # body_gimbal angle
R_x = rotation_about_X(eta_x)  # gimbal_actuator angle

# Initial direction (pointing up)
d_initial = [0, 0, 1]

# Final thrust direction
d_thrust = R_y @ R_x @ d_initial
```

---

## 3. Body Wrench Data Source

### ROS Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `px4/vehicle_thrust_setpoint` | `VehicleThrustSetpoint` | Desired thrust [fx, fy, fz] |
| `px4/vehicle_torque_setpoint` | `VehicleTorqueSetpoint` | Desired torque [tx, ty, tz] |
| `px4/control_allocation_meta_data` | `ControlAllocationMetaData` | Allocation results |

### ControlAllocationMetaData Structure

From `src/px4_msgs/msg/ControlAllocationMetaData.msg`:

```
uint64 timestamp

float32[6] control_sp           # Desired wrench [tx, ty, tz, fx, fy, fz]
float32[6] allocated_control    # Actual allocated wrench

# Pseudo-forces per iteration (4 iterations x 4 modules = 16 values)
float32[16] f_x
float32[16] f_y
float32[16] f_z
float32[16] t_x
float32[16] t_y
float32[16] t_z
float32[16] t_min

int8[4] saturated_idx           # Saturated module indices
float32[4] increment            # Allocation progress
```

### Wrench Calculation

Total body wrench is computed as:

```
Total Force = sum(thrust_i * direction_i) for i in [1,4]
Total Torque = sum(position_i x thrust_vector_i) for i in [1,4]
```

Where:
- `thrust_i`: Magnitude of thrust from module i
- `direction_i`: Unit vector of thrust direction for module i
- `position_i`: Position of module i relative to body center
- `x`: Cross product

### Module Positions

From `src/tvmd/urdf/tvmd.xacro`:

| Module | Position [x, y, z] (m) |
|--------|------------------------|
| 1 | [0.11, 0.11, 0.0] |
| 2 | [-0.11, 0.11, 0.0] |
| 3 | [-0.11, -0.11, 0.0] |
| 4 | [0.11, -0.11, 0.0] |

### Body Inertial Parameters

From `src/tvmd/urdf/Navigator-Def.xacro:7-9`:

| Parameter | Value | Unit |
|-----------|-------|------|
| Mass | 2.83791501 | kg |
| Ixx | 0.01210074 | kg.m^2 |
| Iyy | 0.01841678 | kg.m^2 |
| Izz | 0.02144845 | kg.m^2 |

---

## 4. Log File Format

### ULog Format

The project uses **ULog** (`.ulg`) format, which is PX4's native binary log format.

**Reading ULog files:**
```python
from pyulog import ULog

ulog = ULog("flight_log.ulg")
data = ulog.get_dataset("vehicle_thrust_setpoint").data
timestamp = data["timestamp"]
thrust_x = data["xyz[0]"]
```

**Key Tools:**
- `src/tvmd/Visualizer.py`: Core data reading utilities
- `src/tvmd/Plotter.py`: Plotting utilities
- `tools/ulog2bag.sh`: Convert ULog to ROS bag

### Data Reading Example

From `src/tvmd/draw_attitude_stabilization.py`:

```python
from Visualizer import FlightDataVisualizer

viz = FlightDataVisualizer("log.ulg", "output_folder", t_start, t_end)

# Read thrust/torque setpoints
thrusts, t = viz.read_vector_from_dataset("vehicle_thrust_setpoint", keys=["xyz"])
torques, t = viz.read_vector_from_dataset("vehicle_torque_setpoint", keys=["xyz"])

# Read control allocation results
allocated_wrench, t = viz.read_vector_from_dataset(
    "control_allocation_meta_data",
    keys=["allocated_control"],
    num_entry=6
)

# Read actuator outputs
motor_controls, t = viz.read_vector_from_dataset("actuator_motors", keys=["control"], num_entry=8)
servo_controls, t = viz.read_vector_from_dataset("actuator_servos", keys=["control"], num_entry=8)
```

---

## 5. Test Script

A test script is provided to simulate thrust vector inputs and visualize the resulting joint angles and body wrench:

```bash
cd src/tvmd
python3 test_thrust_to_joint_wrench.py
```

This script:
1. Generates synthetic control inputs
2. Computes corresponding joint angles
3. Calculates body wrench
4. Visualizes all data

---

## 6. Key Files Reference

| Purpose | File Path |
|---------|-----------|
| Joint angle processing | `src/tvmd/republisher.py:367-396` |
| Motor parameters | `src/tvmd/urdf/tvmd.gazebo:38-57` |
| Joint definitions | `src/tvmd/urdf/Single-Agent-Def.xacro` |
| Body parameters | `src/tvmd/urdf/Navigator-Def.xacro` |
| Data visualization | `src/tvmd/Visualizer.py` |
| Plotting utilities | `src/tvmd/Plotter.py` |
| ROS message definitions | `src/px4_msgs/msg/` |
| Test script | `src/tvmd/test_thrust_to_joint_wrench.py` |
