#!/usr/bin/env python3
"""
TVMD Attitude Stabilization Test
Takeoff -> Hover at 1m -> Test attitude:
  Roll  +45° hold 5s, Roll  -45° hold 5s
  Pitch +45° hold 5s, Pitch -45° hold 5s
  Yaw   +90° hold 5s, Yaw   -90° hold 5s
-> Land
"""
import time
import sys
import math
from pymavlink import mavutil

# ============ Configuration ============
FLIGHT_HEIGHT  = 1.0   # Hover altitude (meters)
HOVER_THRUST   = 0.6   # Thrust for level hover (0.0~1.0); tune if needed
TILT_THRUST    = 0.72  # Thrust when tilted 45° (~HOVER_THRUST / cos(45°))
ATTITUDE_HOLD  = 5.0   # Seconds to hold each attitude
TAKEOFF_HOLD   = 3.0   # Seconds to stabilize at hover before tests

# ============ Connect ============
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
master.target_system    = master.target_system
master.target_component = 1
print(f"Connected! (system {master.target_system}, component {master.target_component})")

# ============ Helper functions ============

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (rad) to quaternion [w, x, y, z] (ZYX convention)."""
    cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]

def set_attitude_target(roll, pitch, yaw, thrust):
    """
    Send attitude setpoint via SET_ATTITUDE_TARGET.
    type_mask = 0b00000111: ignore body rates, use quaternion + thrust.
    """
    q = euler_to_quaternion(roll, pitch, yaw)
    master.mav.set_attitude_target_send(
        0,                          # time_boot_ms (0 = use system time)
        master.target_system,
        master.target_component,
        0b00000111,                 # ignore body rates
        q,                          # quaternion [w, x, y, z]
        0.0, 0.0, 0.0,              # body roll/pitch/yaw rate (ignored)
        thrust)                     # normalized thrust 0~1

def set_position_yaw_target(x, y, z, yaw=0.0):
    """Send position + yaw setpoint in NED frame (z negative = up)."""
    type_mask = 0b0000101111111000  # use position + yaw
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
        x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0)

def get_local_position():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if msg:
        return msg.x, msg.y, msg.z
    return None, None, None

def get_attitude():
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        return msg.roll, msg.pitch, msg.yaw
    return None, None, None

def wait_for_mode(target_main_mode, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            main_mode = (msg.custom_mode >> 16) & 0xFF
            if main_mode == target_main_mode:
                return True
    return False

def set_mode_offboard():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        6.0, 0.0, 0.0, 0.0, 0.0, 0.0)

def set_mode_land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        4.0, 6.0, 0.0, 0.0, 0.0, 0.0)

def arm(force=False):
    p2 = 21196.0 if force else 0.0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0, p2, 0.0, 0.0, 0.0, 0.0, 0.0)

def fly_to_hover(x, y, z, yaw=0.0, timeout=20, threshold=0.2):
    """Position-control climb to hover point; returns True when within threshold."""
    start      = time.time()
    last_print = 0.0
    while time.time() - start < timeout:
        set_position_yaw_target(x, y, z, yaw)
        px, py, pz = get_local_position()
        if px is not None:
            dist = math.sqrt((px - x)**2 + (py - y)**2 + (pz - z)**2)
            now  = time.time()
            if now - last_print > 1.0:
                r, p, yw = get_attitude()
                print(f"    pos=({px:.2f},{py:.2f},{pz:.2f})  "
                      f"att=({math.degrees(r or 0):.1f}°,"
                      f"{math.degrees(p or 0):.1f}°,"
                      f"{math.degrees(yw or 0):.1f}°)  dist={dist:.2f}")
                last_print = now
            if dist < threshold:
                return True
        time.sleep(0.05)
    return False

def hold_attitude(roll_deg, pitch_deg, yaw_deg, duration, thrust=None):
    """
    Hold specified attitude (degrees) for `duration` seconds using
    SET_ATTITUDE_TARGET.  Prints status every second.
    """
    roll  = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw   = math.radians(yaw_deg)
    thr   = thrust if thrust is not None else (
                TILT_THRUST if (abs(roll_deg) > 5 or abs(pitch_deg) > 5)
                else HOVER_THRUST)
    end_time   = time.time() + duration
    last_print = 0.0
    while time.time() < end_time:
        set_attitude_target(roll, pitch, yaw, thr)
        now = time.time()
        if now - last_print > 1.0:
            r, p, yw = get_attitude()
            remaining = end_time - now
            print(f"    att=({math.degrees(r or 0):.1f}°,"
                  f"{math.degrees(p or 0):.1f}°,"
                  f"{math.degrees(yw or 0):.1f}°)  "
                  f"remaining={remaining:.1f}s")
            last_print = now
        time.sleep(0.05)

def hold_position_yaw(x, y, z, yaw_deg, duration):
    """Hold position + yaw (degrees) for `duration` seconds."""
    yaw      = math.radians(yaw_deg)
    end_time = time.time() + duration
    last_print = 0.0
    while time.time() < end_time:
        set_position_yaw_target(x, y, z, yaw)
        now = time.time()
        if now - last_print > 1.0:
            r, p, yw = get_attitude()
            print(f"    yaw={math.degrees(yw or 0):.1f}°  "
                  f"remaining={end_time - now:.1f}s")
            last_print = now
        time.sleep(0.05)

# ============ Pre-flight checks ============
print("\nChecking local position estimate...")
px, py, pz = get_local_position()
if px is None:
    print("ERROR: No LOCAL_POSITION_NED. Is EKF2 running?")
    sys.exit(1)
print(f"  Current position: ({px:.2f}, {py:.2f}, {pz:.2f})")

H = -FLIGHT_HEIGHT  # NED: up is negative

# ============ Pre-arm: send setpoints ============
print("\nSending initial setpoints for 5 s (OFFBOARD requirement)...")
for _ in range(100):
    set_position_yaw_target(0.0, 0.0, H, 0.0)
    time.sleep(0.05)

# ============ OFFBOARD mode ============
print("Setting OFFBOARD mode...")
set_mode_offboard()
if not wait_for_mode(6):
    print("  Retrying OFFBOARD...")
    for _ in range(50):
        set_position_yaw_target(0.0, 0.0, H, 0.0)
        time.sleep(0.05)
    set_mode_offboard()
    wait_for_mode(6)
print("  OFFBOARD mode active.")

# ============ Arm ============
print("Arming...")
for _ in range(20):
    set_position_yaw_target(0.0, 0.0, H, 0.0)
    time.sleep(0.05)
arm()

armed = False
for _ in range(100):
    set_position_yaw_target(0.0, 0.0, H, 0.0)
    msg = master.recv_match(type='HEARTBEAT', blocking=False)
    if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(0.05)

if not armed:
    print("  Trying force arm...")
    arm(force=True)
    for _ in range(100):
        set_position_yaw_target(0.0, 0.0, H, 0.0)
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        time.sleep(0.05)

if not armed:
    print("ERROR: Failed to arm.")
    sys.exit(1)
print("  Armed successfully!")

# ============ Takeoff & Hover ============
print("\n" + "="*55)
print("STEP 1: TAKEOFF to (0, 0, -1m) and hover")
print("="*55)
if fly_to_hover(0.0, 0.0, H, yaw=0.0, timeout=25):
    print(f"  Reached hover. Stabilizing for {TAKEOFF_HOLD}s...")
    hold_position_yaw(0.0, 0.0, H, yaw_deg=0.0, duration=TAKEOFF_HOLD)
else:
    print("  WARNING: Takeoff timeout — proceeding anyway.")

# ============ Attitude Test Sequence ============
print("\n" + "="*55)
print("STEP 2: ATTITUDE TESTS  (attitude control, open-loop thrust)")
print("  NOTE: vehicle may drift horizontally during tilt tests.")
print("="*55)

tests = [
    # (roll_deg, pitch_deg, yaw_deg, thrust_override, label)
    ( 45,  0,   0,  None, "Roll  +45°"),
    (-45,  0,   0,  None, "Roll  -45°"),
    (  0,  0,   0,  HOVER_THRUST, "Level  (roll recovery)"),
    (  0, 45,   0,  None, "Pitch +45°"),
    (  0,-45,   0,  None, "Pitch -45°"),
    (  0,  0,   0,  HOVER_THRUST, "Level  (pitch recovery)"),
]

for roll_d, pitch_d, yaw_d, thr, label in tests:
    hold = ATTITUDE_HOLD if "Level" not in label else 2.0
    print(f"\n  [{label}]  roll={roll_d}°  pitch={pitch_d}°  yaw={yaw_d}°  hold={hold}s")
    hold_attitude(roll_d, pitch_d, yaw_d, hold, thrust=thr)

# ============ Yaw Tests (position control — safer for heading) ============
print("\n" + "="*55)
print("STEP 3: YAW TESTS  (position control + yaw setpoint)")
print("="*55)

# Return to origin first
print("\n  [Return to origin before yaw test]")
fly_to_hover(0.0, 0.0, H, yaw=0.0, timeout=20)

yaw_tests = [
    ( 90.0, "Yaw  +90°"),
    (-90.0, "Yaw  -90°"),
    (  0.0, "Yaw   0°  (return)"),
]
for yaw_d, label in yaw_tests:
    print(f"\n  [{label}]  yaw={yaw_d}°  hold={ATTITUDE_HOLD}s")
    # First fly to desired heading
    fly_to_hover(0.0, 0.0, H, yaw=math.radians(yaw_d), timeout=10)
    hold_position_yaw(0.0, 0.0, H, yaw_deg=yaw_d, duration=ATTITUDE_HOLD)

# ============ Land ============
print("\n" + "="*55)
print("LANDING")
print("="*55)
set_mode_land()
time.sleep(15)
print("Done!")
