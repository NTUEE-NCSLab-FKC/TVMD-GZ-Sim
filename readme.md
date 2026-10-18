# TVMD Model Preview and Flight Review

This project establishes basic simulation environment for TVMD in GZ-Sim.

# Features

- Generation of TVMD sdf model (for GZ-Sim)
- Preview of TVMD model in Rviz
- Convert the ulog file to rosbag file, which can be used to replay the flight in Rviz

# Quick Start

To build this project:
```bash
catkin_make
. ./devel/setup.bash
```
Note that if the topics in `src/px4_msgs/msg` are modified, you need to make a clean build.

## Model Preview
Use Rviz to inspect the model:
```bash
roslaunch tvmd display.launch
```
- This will launch Rviz with TVMD model and start the republisher.

## Flight Review

This feature works for SITL and real flight logs.
1. Convert the ulog file to rosbag file using the provided [script](#ulog-file-conversion):
    ```bash
    ./ulog2bag.sh -f <ulog_filename>.ulg
    ```
    This will create a rosbag file in the same directory as the ulog file.
2. Launch rviz with TVMD model and start the republisher:
    ```bash
    roslaunch tvmd display.launch
    ```
    The rosbag file can be published using PlotJuggler, and `display.launch` will start the `republisher` node to initialize Rviz and  subscribe topics in rosbag i.e.,`px4/actuator_motors` and `px4/actuator_servos`. 
    - Note: launch `display.launch` first to start a ROS master for PlotJuggler.
3. Open `PlorJuggler`, go to the `Publisher` tab, click the `ROS Topic Re-Publisher` button, and select the topics to republish. 
    - The two topics are republished to `/joint_states`, which is the control input of the model in Rviz.
    - `px4/vehicle_attitude` and `px4/vehicle_local_position` are republished as the TF `world_frame` -> `base_link` (`world_frame` is North-West-Up, `base_link` is Forward-Left-Up), and the trajectory is published to `/trajectory` (`nav_msgs/Path`).
    - The fixed frame in Rviz is `world_frame`, and the camera follows `base_link`. Set `Views > Target Frame` to `<Fixed Frame>` to keep the camera still.
    - All incoming topics are buffered (only the latest message is kept) and republished at a fixed rate with a common timestamp (`roslaunch tvmd display.launch rate:=50`), so that the modules stay attached to the body while the vehicle moves.


Alternatively, replay the rosbag directly with `rosbag play` and select the time range from the command line:
```bash
roslaunch tvmd playback.launch bag:=/path/to/flight.bag start:=30 duration:=20 speed:=0.5
```
Args: `start` (offset from the beginning of the bag [s]), `duration` (0 for the whole bag), `speed` (playback speed factor), `loop`, `delay` (wait for Rviz [s]). In Rviz, the thrust arrows are the actual thrusts of the propellers (`actuator_motors`, thrust = 6.125 * u, 0.1 m/N, colored with the same red-to-green gradient as the pseudo-forces, or `thrust_color_mode:=magnitude` (blue 0, green 50%, red full throttle) / `thrust_color_mode:=module`), drawn along the propeller axes from the propeller centers, while the colored line strips are the allocated pseudo-forces (`control_allocation_meta_data`, before PTE). The pseudo-forces, wrenches and gimbal angles are low-pass filtered for a smoother view (`force_filter_tau:=0.05 servo_filter_tau:=0.03` [s], set `0` to see the raw data). Pause/resume with `rosservice call /player/pause_playback true` (`false` to resume).

If the firmware is compiled in control allocation debug mode, the `ControlAllocationMetaData` topic will be available in the rosbag file. This topic contains the control allocation pseudo-forces and the calculation intermediate variables, which are helpful in debugging. The `republisher` node will publish this topic and convert them to [/visualization_msgs](https://wiki.ros.org/visualization_msgs), such that this data can be displayed in Rviz.

## Live View (MAVLink)

`live.launch` shows the current state of PX4 (SITL or Pixhawk) in Rviz. `mavlink_bridge.py` converts MAVLink to the same `px4/*` topics as the rosbags, so the republisher and Rviz work in the same way:

| MAVLink | Topic |
|---|---|
| `ATTITUDE_QUATERNION` | `px4/vehicle_attitude` |
| `LOCAL_POSITION_NED` | `px4/vehicle_local_position` |
| `SERVO_OUTPUT_RAW` (motor port) | `px4/actuator_motors` |
| `SERVO_OUTPUT_RAW` (servo port) | `px4/actuator_servos` |

The PWM outputs are converted back to the normalized controls by inverting the PX4 output mapping (output ranges in `config/mavlink_<target>.yaml`). `control_allocation_meta_data` is not available through MAVLink, so only the thrust arrows (not the pseudo-forces) are shown.

Requires `pymavlink` (`pip3 install pymavlink`, installed with MAVProxy).

### SITL

QGC (14550) and MAVROS (14540) already use the default MAVLink links, so start a dedicated one in the PX4 shell (`pxh>`):
```
mavlink start -u 14590 -o 14591 -r 4000000 -m onboard
mavlink stream -u 14590 -s ATTITUDE_QUATERNION -r 50
mavlink stream -u 14590 -s LOCAL_POSITION_NED -r 50
mavlink stream -u 14590 -s SERVO_OUTPUT_RAW_0 -r 50
mavlink stream -u 14590 -s SERVO_OUTPUT_RAW_1 -r 50
```
Then:
```bash
roslaunch tvmd live.launch target:=sitl
```

### Pixhawk (through MAVProxy)

Add an output for the bridge to MAVProxy (keep the outputs of QGC and MAVROS):
```bash
mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14560 ...
roslaunch tvmd live.launch target:=pixhawk
```
Over USB, PX4 streams all the messages at 20-50 Hz. Over a telemetry radio the default rates are low, and `SERVO_OUTPUT_RAW_1` (the servos on AUX) is not streamed: enable them in the MAVLink console of QGC, e.g., `mavlink stream -d /dev/ttyS1 -s SERVO_OUTPUT_RAW_1 -r 20` (the device of the telemetry port).

The bridge logs the received rates every 10 s, and the first `SERVO_OUTPUT_RAW` of each port: check that port 0 holds the motors and port 1 the servos, otherwise swap `motor_port`/`servo_port` in the config.

## Additional Tools
### SDF File Generation
To generate the SDF file for GZ-Sim:
```bash
sh ./tools/gen_sdf.sh
```

### ULog File Conversion
The conversion is done by `tools/pyulog/pyulog/ulog2rosbag.py`. Please follow the instruction in [`tools/pyulog`](./tools/pyulog/README.md) to install the tool.
A script is provided to facilitate the conversion:
```bash
./ulog2bag.sh -f <ulog_filename>.ulg
```
Args: 
+ `-f`: specify the ulog file to convert
+ `-a`: specify the topic to convert (default: `control_allocation_meta_data,actuator_motors,actuator_servos,vehicle_attitude,vehicle_local_position`, etc.)

Note that this feature requires a ROS environment with `px4_msgs` built and sourced. And the `px4_msgs` package should have `ControlAllocationMetaData.msg` in `px4_msgs/msg`, which can be obtained in [TVMD-UAV/TVMD-PX4](https://github.com/TVMD-UAV/TVMD-PX4). 


### Visualization of Flight Logs

```bash
python3 draw_attitude_error.py <path_to_ulog>
```