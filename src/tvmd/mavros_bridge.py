#!/usr/bin/env python3
"""
mavros_bridge.py

Converts MAVROS topics (ROS 1, MAVLink) to the px4_msgs format that
republisher.py expects, enabling real-time RViz visualization from
PX4-SITL without rosbag or PlotJuggler.

Topic mapping:
  /mavros/rc/out              ->  px4/actuator_servos  (channels SERVO_CH_START..SERVO_CH_END)
                                  px4/actuator_motors  (channels MOTOR_CH_START..MOTOR_CH_END)
  /mavros/local_position/pose ->  px4/vehicle_attitude
  (zero-filled)               ->  px4/control_allocation_meta_data

PWM normalisation:
  PX4 SITL outputs rc/out in the range 0-1000 (midpoint 500).
  normalized = (pwm - 500) / 500  ->  [-1.0, 1.0]
  Channels that are 0 (disarmed / unused) are passed through as 0.

Channel layout (adjust to match your TVMD mixer):
  channels[0..3]  -> servos (gimbal / actuator joints)
  channels[4..7]  -> motors (propeller joints)

Coordinate frames:
  MAVROS uses ENU world / FLU body (ROS convention).
  PX4 VehicleAttitude uses NED world / FRD body.
  This bridge converts ENU/FLU -> NED/FRD so that republisher.py's
  vehicle_attitude_listener() produces the correct base_link TF.
"""

import rospy
import tf.transformations as tft
import numpy as np

from mavros_msgs.msg import RCOut
from geometry_msgs.msg import PoseStamped

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import ControlAllocationMetaData

# ── Channel layout ────────────────────────────────────────────────────────────
# Map rc/out channel index -> ActuatorServos.control index.
# republisher.py reads servos as: ctrl[2*i + j] where i=module(0-3), j=0(gimbal_actuator) or 1(body_gimbal)
#
# Assumed TVMD mixer layout (one tilt servo + one motor per module):
#   ch[0..3] = tilt servos for module 1..4  -> servo ctrl[1,3,5,7] (body_gimbal_joint)
#   ch[4..7] = motors for module 1..4       -> motor ctrl[0..3]
#
# SERVO_CH_MAP: list of (rc_out_channel, actuator_servos_control_index)
SERVO_CH_MAP   = [(0, 1), (1, 3), (2, 5), (3, 7)]  # one gimbal per module
MOTOR_CHANNELS = [4, 5, 6, 7]   # -> ActuatorMotors.control[0..3]

# PWM midpoint and half-range for normalisation.
# PX4 SITL uses 0-1000 (mid=500). Change to mid=1500, half=500 for real hardware.
PWM_MID  = 500
PWM_HALF = 500

# Quaternion ENU -> NED  (xyzw): rotate 90° around z then 180° around x
_Q_ENU2NED = np.array([0.70710678, 0.70710678, 0.0, 0.0])


def _pwm_to_norm(pwm):
    """Convert a single PWM value to [-1, 1]. Returns 0.0 for disarmed (pwm==0)."""
    if pwm == 0:
        return 0.0
    return float(pwm - PWM_MID) / PWM_HALF


def enu_flu_to_ned_frd(q_enu_flu_xyzw):
    """
    Convert quaternion from ENU/FLU (MAVROS) to NED/FRD (PX4).
    Returns [w, x, y, z] (PX4 wxyz order).
    """
    # World frame: ENU -> NED
    q_ned_flu = tft.quaternion_multiply(_Q_ENU2NED, q_enu_flu_xyzw)
    # Body frame: FLU -> FRD  (180° around x = [1,0,0,0] in xyzw)
    q_flu2frd = np.array([1.0, 0.0, 0.0, 0.0])
    q_ned_frd = tft.quaternion_multiply(q_ned_flu, q_flu2frd)
    x, y, z, w = q_ned_frd
    return [w, x, y, z]


class MavrosBridge:
    def __init__(self):
        rospy.init_node('mavros_bridge')

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_motors  = rospy.Publisher('px4/actuator_motors',  ActuatorMotors,  queue_size=1)
        self._pub_servos  = rospy.Publisher('px4/actuator_servos',  ActuatorServos,  queue_size=1)
        self._pub_att     = rospy.Publisher('px4/vehicle_attitude', VehicleAttitude, queue_size=1)
        self._pub_meta    = rospy.Publisher('px4/control_allocation_meta_data',
                                            ControlAllocationMetaData, queue_size=1)

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('/mavros/rc/out',             RCOut,        self._rcout_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)

        rospy.loginfo('[mavros_bridge] Ready. Bridging /mavros/rc/out -> px4_msgs.')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _ros_time_us(self):
        return int(rospy.Time.now().to_nsec() // 1000)

    def _zero_meta(self, ts):
        msg = ControlAllocationMetaData()
        msg.timestamp         = ts
        msg.control_sp        = [0.0] * 6
        msg.allocated_control = [0.0] * 6
        msg.f_x   = [0.0] * 16
        msg.f_y   = [0.0] * 16
        msg.f_z   = [0.0] * 16
        msg.t_x   = [0.0] * 16
        msg.t_y   = [0.0] * 16
        msg.t_z   = [0.0] * 16
        msg.t_min = [0.0] * 16
        msg.saturated_idx = [-1] * 4
        msg.increment     = [0.0] * 4
        return msg

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _rcout_cb(self, data: RCOut):
        ts = self._ros_time_us()
        ch = list(data.channels)

        # Pad to at least 8 channels in case fewer arrive
        while len(ch) < 8:
            ch.append(0)

        # ── Servos ────────────────────────────────────────────────────────────
        servo_msg = ActuatorServos()
        servo_msg.timestamp        = ts
        servo_msg.timestamp_sample = ts
        servo_ctrl = [0.0] * 8
        for ch_idx, ctrl_idx in SERVO_CH_MAP:
            servo_ctrl[ctrl_idx] = _pwm_to_norm(ch[ch_idx])
        servo_msg.control = servo_ctrl
        self._pub_servos.publish(servo_msg)

        # ── Motors ────────────────────────────────────────────────────────────
        motor_msg = ActuatorMotors()
        motor_msg.timestamp        = ts
        motor_msg.timestamp_sample = ts
        motor_msg.reversible_flags = 0
        motor_ctrl = [0.0] * 12
        for out_idx, ch_idx in enumerate(MOTOR_CHANNELS):
            motor_ctrl[out_idx] = _pwm_to_norm(ch[ch_idx])
        motor_msg.control = motor_ctrl
        self._pub_motors.publish(motor_msg)

        # ── Meta data (unavailable via MAVLink, publish zeros) ────────────────
        self._pub_meta.publish(self._zero_meta(ts))

        rospy.logdebug('[mavros_bridge] rc/out servos=%s motors=%s',
                       [round(servo_ctrl[i], 3) for i in range(4)],
                       [round(motor_ctrl[i], 3) for i in range(4)])

    def _pose_cb(self, data: PoseStamped):
        o = data.pose.orientation
        q_wxyz = enu_flu_to_ned_frd([o.x, o.y, o.z, o.w])

        ts = self._ros_time_us()
        msg = VehicleAttitude()
        msg.timestamp        = ts
        msg.timestamp_sample = ts
        msg.q                = q_wxyz
        msg.delta_q_reset    = [1.0, 0.0, 0.0, 0.0]
        msg.quat_reset_counter = 0
        self._pub_att.publish(msg)

    # ── Main ──────────────────────────────────────────────────────────────────

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    bridge = MavrosBridge()
    bridge.run()
