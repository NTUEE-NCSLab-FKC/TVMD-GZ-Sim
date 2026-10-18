#!/usr/bin/env python3
"""
MAVLink -> px4_msgs bridge for live visualization in Rviz.

Receives MAVLink from PX4 (SITL or Pixhawk, e.g., through a MAVProxy output) and publishes the
same topics as the rosbags converted from ulog, such that republisher.py works unchanged:

    ATTITUDE_QUATERNION        -> px4/vehicle_attitude
    LOCAL_POSITION_NED         -> px4/vehicle_local_position
    SERVO_OUTPUT_RAW (motors)  -> px4/actuator_motors
    SERVO_OUTPUT_RAW (servos)  -> px4/actuator_servos

The output values (PWM) are converted back to the normalized controls by inverting the PX4 output
mapping (MixingOutput::output_limit_calc_single and FunctionMotors):
    out = value * (max - min) / 2 + (max + min) / 2,  value in [-1, 1]
    motors: value = 2 * x - 1, control = THR_MDL_FAC * x^2 + (1 - THR_MDL_FAC) * x
    servos: value = control

Note: control_allocation_meta_data is not available through MAVLink, so the pseudo-forces
are not shown in the live mode (the thrust arrows are).
"""

import math
import threading

import rospy
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition


def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


def per_channel(value, n):
    """A scalar parameter applies to all channels, a list gives one value per channel."""
    if isinstance(value, (list, tuple)):
        assert len(value) == n, "expected {} values, got {}".format(n, len(value))
        return [float(v) for v in value]
    return [float(value)] * n


def motor_control(out: float, out_min: float, out_max: float, thr_mdl_fac: float = 0.0) -> float:
    """PWM / output value -> actuator_motors.control [0, 1]."""
    if out_max <= out_min:
        return 0.0
    # Disarmed or stopped motors output a value below the minimum
    x = min(max((out - out_min) / (out_max - out_min), 0.0), 1.0)
    if 0.0 < thr_mdl_fac <= 1.0:
        return thr_mdl_fac * x * x + (1.0 - thr_mdl_fac) * x
    return x


def servo_control(out: float, out_min: float, out_max: float, reverse: bool = False) -> float:
    """PWM / output value -> actuator_servos.control [-1, 1]."""
    if out_max <= out_min:
        return 0.0
    value = (out - (out_max + out_min) / 2.0) / ((out_max - out_min) / 2.0)
    value = min(max(value, -1.0), 1.0)
    return -value if reverse else value


def servo_raw(msg, channel: int) -> float:
    """channel: 1-based channel of SERVO_OUTPUT_RAW (servo1_raw ... servo16_raw)."""
    return float(getattr(msg, "servo{}_raw".format(channel)))


class MavlinkBridge:
    def __init__(self) -> None:
        rospy.init_node('tvmd_mavlink_bridge')

        self.url = str(get_param("url", "udpin:0.0.0.0:14591"))
        self.baud = int(get_param("baud", 921600))
        self.request_rate = float(get_param("request_rate", 0.0))   # [Hz], 0: keep the PX4 stream rates

        # Motors (actuator_motors.control[i] <- channel motor_channels[i] of SERVO_OUTPUT_RAW port motor_port)
        self.motor_port = int(get_param("motor_port", 0))
        self.motor_channels = [int(c) for c in get_param("motor_channels", [1, 2, 3, 4])]
        n = len(self.motor_channels)
        self.motor_min = per_channel(get_param("motor_min", 150), n)
        self.motor_max = per_channel(get_param("motor_max", 1000), n)
        self.thr_mdl_fac = float(get_param("thr_mdl_fac", 0.0))

        # Servos (actuator_servos.control[i] <- channel servo_channels[i] of SERVO_OUTPUT_RAW port servo_port)
        self.servo_port = int(get_param("servo_port", 1))
        self.servo_channels = [int(c) for c in get_param("servo_channels", [1, 2, 3, 4, 5, 6, 7, 8])]
        n = len(self.servo_channels)
        self.servo_min = per_channel(get_param("servo_min", 0), n)
        self.servo_max = per_channel(get_param("servo_max", 1000), n)
        self.servo_reverse = [bool(r) for r in per_channel(get_param("servo_reverse", 0), n)]

        self.att_pub = rospy.Publisher("px4/vehicle_attitude", VehicleAttitude, queue_size=10)
        self.lpos_pub = rospy.Publisher("px4/vehicle_local_position", VehicleLocalPosition, queue_size=10)
        self.motor_pub = rospy.Publisher("px4/actuator_motors", ActuatorMotors, queue_size=10)
        self.servo_pub = rospy.Publisher("px4/actuator_servos", ActuatorServos, queue_size=10)

        self.counts = {}
        self.ports_seen = set()
        # SERVO_OUTPUT_RAW.time_usec is uint32 (wraps after ~71 min): unwrap it per port
        self.servo_time = {}

    def unwrap_time_usec(self, port: int, time_usec: int) -> int:
        last, offset = self.servo_time.get(port, (0, 0))
        if time_usec + offset < last - (1 << 31):
            offset += 1 << 32
        self.servo_time[port] = (time_usec + offset, offset)
        return time_usec + offset

    # ---- MAVLink -> px4_msgs ------------------------------------------------------------------

    @staticmethod
    def to_vehicle_attitude(msg) -> VehicleAttitude:
        att = VehicleAttitude()
        att.timestamp = int(msg.time_boot_ms) * 1000
        att.timestamp_sample = att.timestamp
        att.q = [msg.q1, msg.q2, msg.q3, msg.q4]      # w, x, y, z; FRD -> NED, as vehicle_attitude.q
        att.delta_q_reset = [1.0, 0.0, 0.0, 0.0]
        return att

    @staticmethod
    def to_vehicle_local_position(msg) -> VehicleLocalPosition:
        lpos = VehicleLocalPosition()
        lpos.timestamp = int(msg.time_boot_ms) * 1000
        lpos.timestamp_sample = lpos.timestamp
        lpos.xy_valid = True
        lpos.z_valid = True
        lpos.v_xy_valid = True
        lpos.v_z_valid = True
        lpos.x, lpos.y, lpos.z = msg.x, msg.y, msg.z
        lpos.vx, lpos.vy, lpos.vz = msg.vx, msg.vy, msg.vz
        return lpos

    def to_actuator_motors(self, msg, timestamp: int) -> ActuatorMotors:
        motors = ActuatorMotors()
        motors.timestamp = timestamp
        motors.timestamp_sample = motors.timestamp
        control = [float('nan')] * len(motors.control)
        for i, ch in enumerate(self.motor_channels):
            control[i] = motor_control(servo_raw(msg, ch), self.motor_min[i], self.motor_max[i], self.thr_mdl_fac)
        motors.control = control
        return motors

    def to_actuator_servos(self, msg, timestamp: int) -> ActuatorServos:
        servos = ActuatorServos()
        servos.timestamp = timestamp
        servos.timestamp_sample = servos.timestamp
        control = [float('nan')] * len(servos.control)
        for i, ch in enumerate(self.servo_channels):
            control[i] = servo_control(servo_raw(msg, ch), self.servo_min[i], self.servo_max[i], self.servo_reverse[i])
        servos.control = control
        return servos

    def handle(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "ATTITUDE_QUATERNION":
            self.att_pub.publish(MavlinkBridge.to_vehicle_attitude(msg))
        elif mtype == "LOCAL_POSITION_NED":
            self.lpos_pub.publish(MavlinkBridge.to_vehicle_local_position(msg))
        elif mtype == "SERVO_OUTPUT_RAW":
            port = int(getattr(msg, "port", 0))
            if port not in self.ports_seen:
                self.ports_seen.add(port)
                rospy.loginfo("SERVO_OUTPUT_RAW port {}: {}".format(
                    port, [int(servo_raw(msg, ch)) for ch in range(1, 9)]))
            timestamp = self.unwrap_time_usec(port, int(msg.time_usec))
            if port == self.motor_port:
                self.motor_pub.publish(self.to_actuator_motors(msg, timestamp))
            if port == self.servo_port:
                self.servo_pub.publish(self.to_actuator_servos(msg, timestamp))
            mtype = "{}_{}".format(mtype, port)
        else:
            return
        self.counts[mtype] = self.counts.get(mtype, 0) + 1

    # ---- connection ---------------------------------------------------------------------------

    def request_message_rates(self, conn) -> None:
        from pymavlink import mavutil
        interval_us = 1e6 / self.request_rate
        for msg_id in [mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
                       mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                       mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW]:
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, interval_us, 0, 0, 0, 0, 0)

    def report(self, event=None) -> None:
        if not self.counts:
            rospy.logwarn("No MAVLink data received from {} yet".format(self.url))
            return
        expected = ["ATTITUDE_QUATERNION", "LOCAL_POSITION_NED",
                    "SERVO_OUTPUT_RAW_{}".format(self.motor_port), "SERVO_OUTPUT_RAW_{}".format(self.servo_port)]
        rates = ", ".join("{} {:.0f} Hz".format(k, self.counts.get(k, 0) / 10.0) for k in expected)
        missing = [k for k in expected if self.counts.get(k, 0) == 0]
        if missing:
            rospy.logwarn("MAVLink rates (10 s): {}; missing: {}".format(rates, ", ".join(missing)))
        else:
            rospy.loginfo("MAVLink rates (10 s): {}".format(rates))
        self.counts = {}

    def run(self) -> None:
        from pymavlink import mavutil
        rospy.loginfo("Connecting to {}".format(self.url))
        conn = mavutil.mavlink_connection(self.url, baud=self.baud, source_system=255, source_component=191)
        if self.request_rate > 0.0:
            rospy.loginfo("Waiting for a heartbeat to request the message rates...")
            conn.wait_heartbeat(timeout=10)
            self.request_message_rates(conn)
        rospy.Timer(rospy.Duration(10.0), self.report)

        while not rospy.is_shutdown():
            msg = conn.recv_match(type=["ATTITUDE_QUATERNION", "LOCAL_POSITION_NED", "SERVO_OUTPUT_RAW"],
                                  blocking=True, timeout=0.5)
            if msg is not None:
                self.handle(msg)


if __name__ == '__main__':
    MavlinkBridge().run()
