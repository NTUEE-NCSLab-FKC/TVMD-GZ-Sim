#!/usr/bin/env python3
"""
TVMD Attitude Stabilization Test (MAVROS version)
Takeoff -> Hover at 1m -> Test attitude:
  Roll  +45° hold 5s, Roll  -45° hold 5s
  Pitch +45° hold 5s, Pitch -45° hold 5s
  Yaw   +90° hold 5s, Yaw   -90° hold 5s
-> Land

Usage:
  rosrun tvmd attitude_test_mavros.py
  or
  python3 attitude_test_mavros.py
"""
import math
import time
import sys
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State, Thrust
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# ============ Configuration ============
FLIGHT_HEIGHT  = 1.0   # Hover altitude (meters, positive = up in ENU)
HOVER_THRUST   = 0.6   # Normalized thrust for level hover (0.0 ~ 1.0); tune as needed
TILT_THRUST    = 0.72  # Thrust when tilted ±45° (~HOVER_THRUST / cos(45°))
ATTITUDE_HOLD  = 5.0   # Seconds to hold each attitude
TAKEOFF_HOLD   = 3.0   # Seconds to stabilize at hover before tests
POSITION_RATE  = 20    # Hz for position setpoint publishing

# ============ Globals ============
current_state = State()
current_pose  = PoseStamped()


def state_cb(msg):
    global current_state
    current_state = msg


def pose_cb(msg):
    global current_pose
    current_pose = msg


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (rad, ZYX / NED convention) to
    geometry_msgs/Quaternion.
    NOTE: MAVROS uses ENU internally, but attitude setpoints follow
    the same ZYX rotation order, so the quaternion math is identical.
    """
    cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def get_distance(target: PoseStamped) -> float:
    dx = current_pose.pose.position.x - target.pose.position.x
    dy = current_pose.pose.position.y - target.pose.position.y
    dz = current_pose.pose.position.z - target.pose.position.z
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def make_pose(x, y, z, yaw_deg=0.0) -> PoseStamped:
    """Build a PoseStamped in ENU frame (MAVROS local frame)."""
    ps = PoseStamped()
    ps.header.stamp    = rospy.Time.now()
    ps.header.frame_id = "map"
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation = euler_to_quaternion(0, 0, math.radians(yaw_deg))
    return ps


def fly_to_hover(pos_pub, target: PoseStamped,
                 threshold=0.2, timeout=20.0):
    """Publish position setpoint until within threshold or timeout."""
    rate  = rospy.Rate(POSITION_RATE)
    start = rospy.Time.now()
    last_print = rospy.Time(0)

    while not rospy.is_shutdown():
        target.header.stamp = rospy.Time.now()
        pos_pub.publish(target)

        dist = get_distance(target)
        now  = rospy.Time.now()
        if (now - last_print).to_sec() > 1.0:
            p = current_pose.pose.position
            rospy.loginfo(f"  pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})  dist={dist:.2f}")
            last_print = now

        if dist < threshold:
            return True
        if (now - start).to_sec() > timeout:
            rospy.logwarn("fly_to_hover: timeout")
            return False
        rate.sleep()
    return False


def hold_position(pos_pub, target: PoseStamped, duration: float):
    """Hold position setpoint for `duration` seconds."""
    rate     = rospy.Rate(POSITION_RATE)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    last_print = rospy.Time(0)

    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        target.header.stamp = rospy.Time.now()
        pos_pub.publish(target)
        now = rospy.Time.now()
        if (now - last_print).to_sec() > 1.0:
            rem = (end_time - now).to_sec()
            rospy.loginfo(f"  holding... remaining={rem:.1f}s")
            last_print = now
        rate.sleep()


def hold_attitude(att_pub, thr_pub,
                  roll_deg, pitch_deg, yaw_deg,
                  duration: float, thrust: float = None):
    """
    Publish attitude + thrust setpoints for `duration` seconds.
    Publishes to:
      /mavros/setpoint_attitude/attitude  (PoseStamped)
      /mavros/setpoint_attitude/thrust    (Thrust)
    """
    thr = thrust if thrust is not None else (
        TILT_THRUST if (abs(roll_deg) > 5 or abs(pitch_deg) > 5)
        else HOVER_THRUST)

    att_msg = PoseStamped()
    att_msg.header.frame_id = "map"
    att_msg.pose.orientation = euler_to_quaternion(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg))

    thr_msg      = Thrust()
    thr_msg.thrust = thr

    rate     = rospy.Rate(POSITION_RATE)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    last_print = rospy.Time(0)

    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        att_msg.header.stamp = rospy.Time.now()
        thr_msg.header.stamp = rospy.Time.now()
        att_pub.publish(att_msg)
        thr_pub.publish(thr_msg)

        now = rospy.Time.now()
        if (now - last_print).to_sec() > 1.0:
            rem = (end_time - now).to_sec()
            rospy.loginfo(
                f"  att=({roll_deg:.0f}°,{pitch_deg:.0f}°,{yaw_deg:.0f}°) "
                f"thrust={thr:.2f}  remaining={rem:.1f}s")
            last_print = now
        rate.sleep()


# ============ Main ============
def main():
    rospy.init_node("attitude_test_mavros")

    # --- Subscribers ---
    rospy.Subscriber("/mavros/state",               State,       state_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)

    # --- Publishers ---
    pos_pub = rospy.Publisher("/mavros/setpoint_position/local",
                              PoseStamped, queue_size=10)
    att_pub = rospy.Publisher("/mavros/setpoint_attitude/attitude",
                              PoseStamped, queue_size=10)
    thr_pub = rospy.Publisher("/mavros/setpoint_attitude/thrust",
                              Thrust,      queue_size=10)

    # --- Services ---
    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    arming_client   = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode",   SetMode)

    rate = rospy.Rate(POSITION_RATE)

    # ---- Wait for FCU connection ----
    rospy.loginfo("Waiting for FCU connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("FCU connected!")

    # ---- Send setpoints before requesting OFFBOARD ----
    hover_target = make_pose(0.0, 0.0, FLIGHT_HEIGHT, yaw_deg=0.0)
    rospy.loginfo("Pre-sending setpoints for 5 s...")
    for _ in range(100):
        hover_target.header.stamp = rospy.Time.now()
        pos_pub.publish(hover_target)
        rate.sleep()

    # ---- OFFBOARD mode ----
    offboard_req = SetModeRequest()
    offboard_req.custom_mode = "OFFBOARD"
    rospy.loginfo("Requesting OFFBOARD mode...")
    while not rospy.is_shutdown() and current_state.mode != "OFFBOARD":
        if set_mode_client.call(offboard_req).mode_sent:
            rospy.loginfo("  OFFBOARD mode sent.")
        hover_target.header.stamp = rospy.Time.now()
        pos_pub.publish(hover_target)
        rate.sleep()
    rospy.loginfo("  OFFBOARD active.")

    # ---- Arm ----
    arm_req = CommandBoolRequest()
    arm_req.value = True
    rospy.loginfo("Arming...")
    while not rospy.is_shutdown() and not current_state.armed:
        if arming_client.call(arm_req).success:
            rospy.loginfo("  Armed!")
        hover_target.header.stamp = rospy.Time.now()
        pos_pub.publish(hover_target)
        rate.sleep()

    # ============ Takeoff & Hover ============
    rospy.loginfo("="*55)
    rospy.loginfo("STEP 1: TAKEOFF to (0, 0, %.1fm)" % FLIGHT_HEIGHT)
    rospy.loginfo("="*55)
    hover_target = make_pose(0.0, 0.0, FLIGHT_HEIGHT, yaw_deg=0.0)
    if fly_to_hover(pos_pub, hover_target, threshold=0.2, timeout=25):
        rospy.loginfo(f"  Reached hover. Stabilizing {TAKEOFF_HOLD}s...")
        hold_position(pos_pub, hover_target, TAKEOFF_HOLD)
    else:
        rospy.logwarn("  Takeoff timeout — continuing anyway.")

    # ============ Attitude Tests ============
    rospy.loginfo("="*55)
    rospy.loginfo("STEP 2: ATTITUDE TESTS")
    rospy.loginfo("  (open-loop thrust; vehicle may drift horizontally)")
    rospy.loginfo("="*55)

    # (roll_deg, pitch_deg, yaw_deg, thrust_override, label)
    attitude_tests = [
        ( 45,  0,  0,  None,         "Roll  +45°"),
        (-45,  0,  0,  None,         "Roll  -45°"),
        (  0,  0,  0,  HOVER_THRUST, "Level  (roll recovery, 2s)"),
        (  0, 45,  0,  None,         "Pitch +45°"),
        (  0,-45,  0,  None,         "Pitch -45°"),
        (  0,  0,  0,  HOVER_THRUST, "Level  (pitch recovery, 2s)"),
    ]

    for roll_d, pitch_d, yaw_d, thr, label in attitude_tests:
        dur = ATTITUDE_HOLD if "recovery" not in label else 2.0
        rospy.loginfo(
            f"\n  [{label}]  roll={roll_d}°  pitch={pitch_d}°  "
            f"yaw={yaw_d}°  hold={dur}s")
        hold_attitude(att_pub, thr_pub,
                      roll_d, pitch_d, yaw_d, dur, thrust=thr)

    # ============ Yaw Tests (position control) ============
    rospy.loginfo("="*55)
    rospy.loginfo("STEP 3: YAW TESTS  (position control + yaw setpoint)")
    rospy.loginfo("="*55)

    rospy.loginfo("\n  [Return to origin before yaw test]")
    hover_target = make_pose(0.0, 0.0, FLIGHT_HEIGHT, yaw_deg=0.0)
    fly_to_hover(pos_pub, hover_target, threshold=0.3, timeout=20)

    yaw_tests = [
        ( 90.0, "Yaw  +90°"),
        (-90.0, "Yaw  -90°"),
        (  0.0, "Yaw    0°  (return)"),
    ]
    for yaw_d, label in yaw_tests:
        rospy.loginfo(f"\n  [{label}]  yaw={yaw_d}°  hold={ATTITUDE_HOLD}s")
        target = make_pose(0.0, 0.0, FLIGHT_HEIGHT, yaw_deg=yaw_d)
        fly_to_hover(pos_pub, target, threshold=0.1, timeout=10)
        hold_position(pos_pub, target, ATTITUDE_HOLD)

    # ============ Land ============
    rospy.loginfo("="*55)
    rospy.loginfo("LANDING")
    rospy.loginfo("="*55)
    land_req = SetModeRequest()
    land_req.custom_mode = "AUTO.LAND"
    set_mode_client.call(land_req)
    rospy.loginfo("AUTO.LAND sent. Done!")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
