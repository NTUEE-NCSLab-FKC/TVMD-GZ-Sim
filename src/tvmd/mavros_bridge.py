#!/usr/bin/env python3
"""
mavros_bridge.py

Converts MAVROS topics (ROS 1, MAVLink) to the px4_msgs format that
republisher.py expects, enabling real-time RViz visualization from
PX4-SITL without rosbag or PlotJuggler.

Topic mapping:
  /mavros/target_actuator_control  ->  px4/actuator_motors
                                       px4/actuator_servos
  /mavros/local_position/pose      ->  px4/vehicle_attitude
  (zero-filled)                    ->  px4/control_allocation_meta_data

Coordinate frames:
  MAVROS uses ENU world / FLU body (ROS convention).
  PX4 VehicleAttitude uses NED world / FRD body.
  This bridge converts ENU/FLU -> NED/FRD so that republisher.py's
  vehicle_attitude_listener() produces the correct TF output.

Actuator group convention (PX4 default mixer for TVMD-style airframes):
  group 0: servos  (controls[0..7], first 8 outputs)
  group 1: motors  (controls[0..7], first 8 outputs)
  Adjust GROUP_SERVOS / GROUP_MOTORS below if your mixer differs.
"""

import rospy
import tf.transformations as tft
import numpy as np

from mavros_msgs.msg import ActuatorControl
from geometry_msgs.msg import PoseStamped

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import ControlAllocationMetaData

# ── Tunable parameters ────────────────────────────────────────────────────────
# MAVLink actuator group indices used by the TVMD mixer.
# Check your PX4 mixer file if the joints look wrong.
GROUP_SERVOS = 0   # which group_mix value carries servo (gimbal) outputs
GROUP_MOTORS = 1   # which group_mix value carries motor outputs

# Fixed quaternion that rotates FLU body -> FRD body (180° around x).
# Applied when converting MAVROS orientation to PX4 convention.
_Q_FLU2FRD = np.array([1.0, 0.0, 0.0, 0.0])   # xyzw: [sin(π/2),0,0,cos(π/2)] ... actually 180° around x

# Quaternion: NED world -> ENU world  (used to invert MAVROS frame)
# 90° around z followed by 180° around x, expressed as a single quaternion (xyzw).
_Q_ENU2NED = np.array([0.70710678, 0.70710678, 0.0, 0.0])  # xyzw


def enu_flu_to_ned_frd(q_enu_flu_xyzw):
    """
    Convert a quaternion expressed in ENU world / FLU body (MAVROS) to
    NED world / FRD body (PX4 VehicleAttitude convention).

    Returns quaternion in wxyz order (PX4 convention).
    """
    # Step 1: world frame  ENU -> NED
    #   q_ned_body = q_enu2ned * q_enu_body
    q_ned_flu = tft.quaternion_multiply(_Q_ENU2NED, q_enu_flu_xyzw)

    # Step 2: body frame  FLU -> FRD  (180° rotation around x)
    q_flu2frd = np.array([1.0, 0.0, 0.0, 0.0])   # xyzw: 180° around x = [1,0,0,0]
    q_ned_frd = tft.quaternion_multiply(q_ned_flu, q_flu2frd)

    # Convert xyzw -> wxyz for PX4
    x, y, z, w = q_ned_frd
    return [w, x, y, z]


class MavrosBridge:
    def __init__(self):
        rospy.init_node('mavros_bridge')

        # ── Publishers (px4_msgs topics that republisher.py subscribes to) ──
        self._pub_motors = rospy.Publisher(
            'px4/actuator_motors', ActuatorMotors, queue_size=1)
        self._pub_servos = rospy.Publisher(
            'px4/actuator_servos', ActuatorServos, queue_size=1)
        self._pub_attitude = rospy.Publisher(
            'px4/vehicle_attitude', VehicleAttitude, queue_size=1)
        self._pub_meta = rospy.Publisher(
            'px4/control_allocation_meta_data', ControlAllocationMetaData, queue_size=1)

        # Latest actuator message cache (group 0 & 1 arrive as separate messages)
        self._latest_servos_ctrl = None
        self._latest_motors_ctrl = None

        # ── Subscribers ──────────────────────────────────────────────────────
        rospy.Subscriber(
            '/mavros/target_actuator_control',
            ActuatorControl,
            self._actuator_cb)
        rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self._pose_cb)

        rospy.loginfo('[mavros_bridge] Ready. Bridging MAVROS -> px4_msgs.')

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _ros_time_us(self):
        return int(rospy.Time.now().to_nsec() // 1000)

    def _publish_zero_meta(self):
        """Publish an all-zero ControlAllocationMetaData so republisher.py
        keeps running even though this data is unavailable via MAVLink."""
        msg = ControlAllocationMetaData()
        msg.timestamp = self._ros_time_us()
        msg.control_sp       = [0.0] * 6
        msg.allocated_control = [0.0] * 6
        msg.f_x  = [0.0] * 16
        msg.f_y  = [0.0] * 16
        msg.f_z  = [0.0] * 16
        msg.t_x  = [0.0] * 16
        msg.t_y  = [0.0] * 16
        msg.t_z  = [0.0] * 16
        msg.t_min = [0.0] * 16
        msg.saturated_idx = [-1] * 4
        msg.increment = [0.0] * 4
        self._pub_meta.publish(msg)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _actuator_cb(self, data: ActuatorControl):
        """
        /mavros/target_actuator_control  (mavros_msgs/ActuatorControl)

        PX4 sends one message per actuator group.  We buffer each group and
        forward it as the appropriate px4_msgs type.

        ActuatorControl fields:
          header       – stamp
          group_mix    – which mixing group this message belongs to (uint8)
          controls[8]  – normalised outputs [-1 .. 1]
        """
        ts = self._ros_time_us()

        if data.group_mix == GROUP_SERVOS:
            msg = ActuatorServos()
            msg.timestamp        = ts
            msg.timestamp_sample = ts
            # controls[8]; pad with zeros if fewer than 8 values arrive
            ctrl = list(data.controls) + [0.0] * (8 - len(data.controls))
            msg.control = ctrl[:8]
            self._pub_servos.publish(msg)
            self._latest_servos_ctrl = msg

        elif data.group_mix == GROUP_MOTORS:
            msg = ActuatorMotors()
            msg.timestamp        = ts
            msg.timestamp_sample = ts
            msg.reversible_flags = 0
            ctrl = list(data.controls) + [0.0] * (12 - len(data.controls))
            msg.control = ctrl[:12]
            self._pub_motors.publish(msg)
            self._latest_motors_ctrl = msg

        # Publish zero meta-data every time actuator data arrives so
        # republisher.py's marker publisher keeps ticking.
        self._publish_zero_meta()

    def _pose_cb(self, data: PoseStamped):
        """
        /mavros/local_position/pose  (geometry_msgs/PoseStamped)

        Orientation is in ENU world / FLU body (MAVROS convention).
        Convert to NED world / FRD body for PX4 VehicleAttitude.
        """
        o = data.pose.orientation
        q_enu_flu = [o.x, o.y, o.z, o.w]   # xyzw

        q_wxyz = enu_flu_to_ned_frd(q_enu_flu)  # wxyz (PX4 order)

        ts = self._ros_time_us()
        msg = VehicleAttitude()
        msg.timestamp        = ts
        msg.timestamp_sample = ts
        msg.q                = q_wxyz           # [w, x, y, z]
        msg.delta_q_reset    = [1.0, 0.0, 0.0, 0.0]  # identity → no reset
        msg.quat_reset_counter = 0
        self._pub_attitude.publish(msg)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    bridge = MavrosBridge()
    bridge.run()
