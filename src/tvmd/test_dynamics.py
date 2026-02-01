#!/usr/bin/env python3
"""
推力向量動力學測試：模擬推力向量變化對機體位置和姿態的影響

此腳本包含簡化的剛體動力學模擬：
- 推力 → 加速度 → 速度 → 位置
- 力矩 → 角加速度 → 角速度 → 姿態

使用方式:
    roslaunch tvmd test_dynamics.launch

Author: Auto-generated for TVMD-GZ-Sim
"""

import rospy
import numpy as np
from scipy.spatial.transform import Rotation

import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import ControlAllocationMetaData
from px4_msgs.msg import VehicleAttitude


# ============================================================================
# 系統參數
# ============================================================================

NUM_MODULES = 4

# 關節限制
JOINT_LIMITS = {
    'gimbal_actuator': {'min': -0.349, 'max': 0.349},
    'body_gimbal': {'min': -np.pi/4, 'max': np.pi/4},
}

# 馬達參數
MOTOR_CONSTANT = 2e-05
MAX_ROT_VELOCITY = 1500
MAX_THRUST = MOTOR_CONSTANT * MAX_ROT_VELOCITY**2

# 模組位置
MODULE_POSITIONS = np.array([
    [0.11, 0.11, 0.0],
    [-0.11, 0.11, 0.0],
    [-0.11, -0.11, 0.0],
    [0.11, -0.11, 0.0],
])

# 機體參數 (來自 Navigator-Def.xacro)
BODY_MASS = 2.83791501  # kg
GRAVITY = 9.81
HOVER_THRUST = BODY_MASS * GRAVITY

# 慣性矩陣 (來自 Navigator-Def.xacro)
INERTIA = np.array([
    [0.01210074, 0.00034820, -0.00023145],
    [0.00034820, 0.01841678, -0.00017235],
    [-0.00023145, -0.00017235, 0.02144845]
])
INERTIA_INV = np.linalg.inv(INERTIA)


# ============================================================================
# 輔助函數
# ============================================================================

def thrust_vector_to_joint_angles(thrust_vector: np.ndarray) -> dict:
    """從推力向量反算關節角度"""
    fx, fy, fz = thrust_vector
    magnitude = np.linalg.norm(thrust_vector)

    if magnitude < 1e-6:
        return {'eta_x': 0.0, 'eta_y': 0.0, 'thrust_magnitude': 0.0}

    dx, dy, dz = thrust_vector / magnitude
    eta_x = np.arcsin(np.clip(-dy, -1, 1))
    cos_eta_x = np.cos(eta_x)
    if abs(cos_eta_x) > 1e-6:
        eta_y = np.arctan2(dx / cos_eta_x, dz / cos_eta_x)
    else:
        eta_y = 0.0

    eta_x = np.clip(eta_x, JOINT_LIMITS['gimbal_actuator']['min'],
                    JOINT_LIMITS['gimbal_actuator']['max'])
    eta_y = np.clip(eta_y, JOINT_LIMITS['body_gimbal']['min'],
                    JOINT_LIMITS['body_gimbal']['max'])

    return {'eta_x': eta_x, 'eta_y': eta_y, 'thrust_magnitude': magnitude}


def joint_angle_to_control(angle: float, joint_type: str) -> float:
    limits = JOINT_LIMITS[joint_type]
    factor = (limits['max'] - limits['min']) / 2
    offset = (limits['max'] + limits['min']) / 2
    return (angle - offset) / factor


def thrust_to_motor_control(thrust: float) -> float:
    if thrust <= 0:
        return 0.0
    omega = np.sqrt(thrust / MOTOR_CONSTANT)
    return np.clip(omega / MAX_ROT_VELOCITY, 0.0, 1.0)


def compute_body_wrench(thrust_vectors: list) -> dict:
    """計算機體座標系下的 wrench"""
    total_force = np.zeros(3)
    total_torque = np.zeros(3)

    for i, thrust_vec in enumerate(thrust_vectors):
        thrust_vec = np.array(thrust_vec)
        total_force += thrust_vec
        total_torque += np.cross(MODULE_POSITIONS[i], thrust_vec)

    return {'force': total_force, 'torque': total_torque}


# ============================================================================
# 剛體動力學模擬器
# ============================================================================

class RigidBodyDynamics:
    """簡化的剛體動力學模擬"""

    def __init__(self):
        # 狀態變數 (世界座標系)
        self.position = np.array([0.0, 0.0, 1.0])  # 初始高度 1m
        self.velocity = np.zeros(3)
        self.orientation = Rotation.identity()  # 四元數
        self.angular_velocity = np.zeros(3)  # 機體座標系

        # 物理參數
        self.mass = BODY_MASS
        self.inertia = INERTIA
        self.inertia_inv = INERTIA_INV
        self.gravity = np.array([0, 0, -GRAVITY])

        # 阻尼係數 (簡化空氣阻力)
        self.linear_damping = 0.1
        self.angular_damping = 0.5

    def reset(self):
        """重置狀態"""
        self.position = np.array([0.0, 0.0, 1.0])
        self.velocity = np.zeros(3)
        self.orientation = Rotation.identity()
        self.angular_velocity = np.zeros(3)

    def step(self, body_force: np.ndarray, body_torque: np.ndarray, dt: float):
        """
        前進一個時間步

        Args:
            body_force: 機體座標系下的力 [N]
            body_torque: 機體座標系下的力矩 [N.m]
            dt: 時間步長 [s]
        """
        # 將機體座標系的力轉換到世界座標系
        R = self.orientation.as_matrix()
        world_force = R @ body_force

        # 計算世界座標系下的總力 (推力 + 重力)
        total_force = world_force + self.mass * self.gravity

        # 加入線性阻尼
        total_force -= self.linear_damping * self.velocity

        # 線性運動 (歐拉積分)
        acceleration = total_force / self.mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        # 地面碰撞檢測
        if self.position[2] < 0:
            self.position[2] = 0
            self.velocity[2] = max(0, self.velocity[2])

        # 角運動 (機體座標系)
        # τ = I·α + ω × (I·ω)
        # α = I⁻¹ · (τ - ω × (I·ω))
        gyroscopic = np.cross(self.angular_velocity, self.inertia @ self.angular_velocity)
        angular_accel = self.inertia_inv @ (body_torque - gyroscopic)

        # 加入角阻尼
        angular_accel -= self.angular_damping * self.angular_velocity

        self.angular_velocity += angular_accel * dt

        # 更新姿態 (四元數積分)
        omega_mag = np.linalg.norm(self.angular_velocity)
        if omega_mag > 1e-10:
            axis = self.angular_velocity / omega_mag
            angle = omega_mag * dt
            delta_rotation = Rotation.from_rotvec(axis * angle)
            self.orientation = self.orientation * delta_rotation

    def get_quaternion(self) -> np.ndarray:
        """返回 [w, x, y, z] 格式的四元數"""
        q = self.orientation.as_quat()  # [x, y, z, w]
        return np.array([q[3], q[0], q[1], q[2]])  # [w, x, y, z]

    def get_euler(self) -> np.ndarray:
        """返回歐拉角 [roll, pitch, yaw] (rad)"""
        return self.orientation.as_euler('xyz')


# ============================================================================
# ROS 發布器
# ============================================================================

class DynamicsTestPublisher:
    """動力學測試發布器"""

    def __init__(self):
        rospy.init_node('test_dynamics', anonymous=True)

        # Publishers
        self.servo_pub = rospy.Publisher('px4/actuator_servos', ActuatorServos, queue_size=10)
        self.motor_pub = rospy.Publisher('px4/actuator_motors', ActuatorMotors, queue_size=10)
        self.meta_pub = rospy.Publisher('px4/control_allocation_meta_data', ControlAllocationMetaData, queue_size=10)
        self.attitude_pub = rospy.Publisher('px4/vehicle_attitude', VehicleAttitude, queue_size=10)

        # TF broadcaster for position
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.rate = rospy.get_param('~rate', 100)  # 100 Hz for dynamics
        self.dt = 1.0 / self.rate

        # 動力學模擬器
        self.dynamics = RigidBodyDynamics()

        # 推力向量設定
        hover_per_module = HOVER_THRUST / 4
        self.thrust_vectors = [
            [0, 0, hover_per_module],
            [0, 0, hover_per_module],
            [0, 0, hover_per_module],
            [0, 0, hover_per_module],
        ]

        rospy.loginfo("Dynamics Test Publisher initialized")
        rospy.loginfo(f"Simulation rate: {self.rate} Hz, dt: {self.dt:.4f} s")
        rospy.loginfo(f"Body mass: {BODY_MASS:.2f} kg")
        rospy.loginfo(f"Hover thrust: {HOVER_THRUST:.2f} N ({hover_per_module:.2f} N per module)")

    def get_timestamp(self):
        return int(rospy.Time.now().to_sec() * 1e6)

    def set_thrust_vectors(self, vectors: list):
        """設定推力向量"""
        self.thrust_vectors = [np.array(v) for v in vectors]

    def publish_tf(self):
        """發布機體位置到 TF"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.dynamics.position[0]
        t.transform.translation.y = self.dynamics.position[1]
        t.transform.translation.z = self.dynamics.position[2]

        q = self.dynamics.get_quaternion()  # [w, x, y, z]
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_actuators(self):
        """發布執行器控制"""
        timestamp = self.get_timestamp()

        servo_controls = [0.0] * 8
        motor_controls = [0.0] * 12

        for i, thrust_vec in enumerate(self.thrust_vectors):
            result = thrust_vector_to_joint_angles(thrust_vec)
            servo_controls[2*i] = joint_angle_to_control(result['eta_x'], 'gimbal_actuator')
            servo_controls[2*i + 1] = joint_angle_to_control(result['eta_y'], 'body_gimbal')
            motor_controls[i] = thrust_to_motor_control(result['thrust_magnitude'])

        servo_msg = ActuatorServos()
        servo_msg.timestamp = timestamp
        servo_msg.control = servo_controls
        self.servo_pub.publish(servo_msg)

        motor_msg = ActuatorMotors()
        motor_msg.timestamp = timestamp
        motor_msg.control = motor_controls
        self.motor_pub.publish(motor_msg)

    def publish_wrench(self, wrench: dict):
        """發布 wrench"""
        meta_msg = ControlAllocationMetaData()
        meta_msg.timestamp = self.get_timestamp()

        meta_msg.allocated_control[0] = wrench['torque'][0]
        meta_msg.allocated_control[1] = wrench['torque'][1]
        meta_msg.allocated_control[2] = wrench['torque'][2]
        meta_msg.allocated_control[3] = wrench['force'][0]
        meta_msg.allocated_control[4] = wrench['force'][1]
        meta_msg.allocated_control[5] = wrench['force'][2]
        meta_msg.control_sp = meta_msg.allocated_control

        for i, thrust_vec in enumerate(self.thrust_vectors):
            for j in range(4):
                idx = j * 4 + i
                scale = (j + 1) / 4.0
                meta_msg.f_x[idx] = thrust_vec[0] * scale
                meta_msg.f_y[idx] = thrust_vec[1] * scale
                meta_msg.f_z[idx] = thrust_vec[2] * scale

        meta_msg.saturated_idx = [-1, -1, -1, -1]
        meta_msg.increment = [0.25, 0.25, 0.25, 0.25]

        self.meta_pub.publish(meta_msg)

    def publish_attitude(self):
        """發布姿態"""
        msg = VehicleAttitude()
        msg.timestamp = self.get_timestamp()

        q = self.dynamics.get_quaternion()  # [w, x, y, z]
        msg.q = [q[0], q[1], q[2], q[3]]
        msg.delta_q_reset = [1.0, 0.0, 0.0, 0.0]

        self.attitude_pub.publish(msg)

    def log_state(self):
        """打印當前狀態"""
        euler = np.degrees(self.dynamics.get_euler())
        rospy.loginfo_throttle(1.0,
            f"Pos: [{self.dynamics.position[0]:.2f}, {self.dynamics.position[1]:.2f}, {self.dynamics.position[2]:.2f}] m | "
            f"RPY: [{euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}]°")

    def run_demo(self):
        """運行動力學演示"""
        rate = rospy.Rate(self.rate)
        start_time = rospy.Time.now()

        hover = HOVER_THRUST / 4

        # 測試案例
        test_cases = [
            {
                'name': 'Hover (懸停平衡)',
                'vectors': [[0, 0, hover], [0, 0, hover], [0, 0, hover], [0, 0, hover]],
                'duration': 3.0
            },
            {
                'name': 'Ascend (上升)',
                'vectors': [[0, 0, hover*1.3], [0, 0, hover*1.3], [0, 0, hover*1.3], [0, 0, hover*1.3]],
                'duration': 2.0
            },
            {
                'name': 'Hover (維持高度)',
                'vectors': [[0, 0, hover], [0, 0, hover], [0, 0, hover], [0, 0, hover]],
                'duration': 2.0
            },
            {
                'name': 'Roll Right (右滾)',
                'vectors': [[1.5, 0, hover], [-1.5, 0, hover], [-1.5, 0, hover], [1.5, 0, hover]],
                'duration': 1.5
            },
            {
                'name': 'Roll Left (左滾回正)',
                'vectors': [[-1.5, 0, hover], [1.5, 0, hover], [1.5, 0, hover], [-1.5, 0, hover]],
                'duration': 1.5
            },
            {
                'name': 'Hover (穩定)',
                'vectors': [[0, 0, hover], [0, 0, hover], [0, 0, hover], [0, 0, hover]],
                'duration': 2.0
            },
            {
                'name': 'Pitch Forward (前傾)',
                'vectors': [[0, 1.5, hover], [0, 1.5, hover], [0, -1.5, hover], [0, -1.5, hover]],
                'duration': 1.5
            },
            {
                'name': 'Pitch Back (後傾回正)',
                'vectors': [[0, -1.5, hover], [0, -1.5, hover], [0, 1.5, hover], [0, 1.5, hover]],
                'duration': 1.5
            },
            {
                'name': 'Descend (下降)',
                'vectors': [[0, 0, hover*0.7], [0, 0, hover*0.7], [0, 0, hover*0.7], [0, 0, hover*0.7]],
                'duration': 3.0
            },
        ]

        rospy.loginfo("=" * 60)
        rospy.loginfo("Starting dynamics simulation demo")
        rospy.loginfo("Watch the robot move in RViz!")
        rospy.loginfo("=" * 60)

        case_idx = 0
        case_start = rospy.Time.now()

        while not rospy.is_shutdown():
            current_case = test_cases[case_idx]
            elapsed = (rospy.Time.now() - case_start).to_sec()

            if elapsed >= current_case['duration']:
                case_idx = (case_idx + 1) % len(test_cases)
                case_start = rospy.Time.now()
                current_case = test_cases[case_idx]

                rospy.loginfo("")
                rospy.loginfo(f">>> Phase {case_idx + 1}: {current_case['name']}")

                # 重置動力學狀態（當回到第一個案例時）
                if case_idx == 0:
                    rospy.loginfo("Resetting simulation...")
                    self.dynamics.reset()

            # 設定推力向量
            self.set_thrust_vectors(current_case['vectors'])

            # 計算 wrench
            wrench = compute_body_wrench(self.thrust_vectors)

            # 動力學模擬步進
            self.dynamics.step(wrench['force'], wrench['torque'], self.dt)

            # 發布所有數據
            self.publish_tf()
            self.publish_actuators()
            self.publish_wrench(wrench)
            self.publish_attitude()

            # 打印狀態
            self.log_state()

            rate.sleep()


def main():
    try:
        publisher = DynamicsTestPublisher()
        publisher.run_demo()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
