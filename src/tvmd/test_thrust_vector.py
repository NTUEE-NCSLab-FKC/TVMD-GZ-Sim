#!/usr/bin/env python3
"""
推力向量測試器：給定各模組的推力向量，計算對應的關節角度和機體 wrench

使用方式:
1. 啟動 RViz: roslaunch tvmd display.launch
2. 執行此腳本: rosrun tvmd test_thrust_vector.py

或直接:
   roslaunch tvmd test_thrust_vector.launch

此腳本的計算流程:
   輸入: 4個模組的推力向量 [fx, fy, fz] (N)
      ↓
   計算: 關節角度 (eta_x, eta_y) 和推力大小
      ↓
   計算: 機體總 wrench (force + torque)
      ↓
   發布: 到 RViz 顯示
"""

import rospy
import numpy as np

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import ControlAllocationMetaData
from sensor_msgs.msg import JointState


# ============================================================================
# 系統參數 (來自 URDF)
# ============================================================================

NUM_MODULES = 4

# 關節限制
JOINT_LIMITS = {
    'gimbal_actuator': {'min': -0.349, 'max': 0.349},  # X軸, ±20°
    'body_gimbal': {'min': -np.pi/4, 'max': np.pi/4},  # Y軸, ±45°
}

# 馬達參數
MOTOR_CONSTANT = 2e-05  # N/(rad/s)²
MAX_ROT_VELOCITY = 1500  # rad/s
MAX_THRUST = MOTOR_CONSTANT * MAX_ROT_VELOCITY**2  # 約 45N

# 模組位置 (相對於 base_link)
MODULE_POSITIONS = np.array([
    [0.11, 0.11, 0.0],    # Module 1
    [-0.11, 0.11, 0.0],   # Module 2
    [-0.11, -0.11, 0.0],  # Module 3
    [0.11, -0.11, 0.0],   # Module 4
])

# 機體參數
BODY_MASS = 2.83791501
GRAVITY = 9.81
HOVER_THRUST = BODY_MASS * GRAVITY  # 約 27.8N


# ============================================================================
# 核心計算函數
# ============================================================================

def thrust_vector_to_joint_angles(thrust_vector: np.ndarray) -> dict:
    """
    從推力向量反算關節角度

    推力向量定義: [fx, fy, fz] 在模組的 body_link 座標系下
    初始推力方向: [0, 0, 1] (向上)

    旋轉順序: 先 body_gimbal (Y軸), 再 gimbal_actuator (X軸)

    Args:
        thrust_vector: 推力向量 [fx, fy, fz] (N)

    Returns:
        {
            'eta_x': gimbal_actuator 角度 (rad),
            'eta_y': body_gimbal 角度 (rad),
            'thrust_magnitude': 推力大小 (N),
            'direction': 推力方向單位向量
        }
    """
    fx, fy, fz = thrust_vector
    magnitude = np.linalg.norm(thrust_vector)

    if magnitude < 1e-6:
        return {
            'eta_x': 0.0,
            'eta_y': 0.0,
            'thrust_magnitude': 0.0,
            'direction': np.array([0, 0, 1])
        }

    # 歸一化方向
    dx, dy, dz = thrust_vector / magnitude

    # 從推力方向反算角度
    # 旋轉順序: R = Ry(eta_y) @ Rx(eta_x)
    # d = R @ [0, 0, 1]^T
    #
    # 展開得:
    # dx = sin(eta_y) * cos(eta_x)
    # dy = -sin(eta_x)
    # dz = cos(eta_y) * cos(eta_x)

    # 從 dy 求 eta_x
    eta_x = np.arcsin(-dy)

    # 從 dx, dz 求 eta_y
    cos_eta_x = np.cos(eta_x)
    if abs(cos_eta_x) > 1e-6:
        sin_eta_y = dx / cos_eta_x
        cos_eta_y = dz / cos_eta_x
        eta_y = np.arctan2(sin_eta_y, cos_eta_y)
    else:
        eta_y = 0.0

    # 限制在關節範圍內
    eta_x = np.clip(eta_x, JOINT_LIMITS['gimbal_actuator']['min'],
                    JOINT_LIMITS['gimbal_actuator']['max'])
    eta_y = np.clip(eta_y, JOINT_LIMITS['body_gimbal']['min'],
                    JOINT_LIMITS['body_gimbal']['max'])

    return {
        'eta_x': eta_x,
        'eta_y': eta_y,
        'thrust_magnitude': magnitude,
        'direction': np.array([dx, dy, dz])
    }


def joint_angle_to_control(angle: float, joint_type: str) -> float:
    """
    關節角度轉換為控制值 [-1, 1]

    control = (angle - offset) / factor
    """
    limits = JOINT_LIMITS[joint_type]
    factor = (limits['max'] - limits['min']) / 2
    offset = (limits['max'] + limits['min']) / 2
    return (angle - offset) / factor


def thrust_to_motor_control(thrust: float) -> float:
    """
    推力轉換為馬達控制值 [0, 1]

    thrust = motorConstant * (control * maxRotVelocity)²
    control = sqrt(thrust / motorConstant) / maxRotVelocity
    """
    if thrust <= 0:
        return 0.0
    omega = np.sqrt(thrust / MOTOR_CONSTANT)
    control = omega / MAX_ROT_VELOCITY
    return np.clip(control, 0.0, 1.0)


def compute_body_wrench(thrust_vectors: list) -> dict:
    """
    計算機體的總 wrench

    Args:
        thrust_vectors: 4個模組的推力向量 [[fx,fy,fz], ...]

    Returns:
        {
            'force': [Fx, Fy, Fz],
            'torque': [Tx, Ty, Tz]
        }
    """
    total_force = np.zeros(3)
    total_torque = np.zeros(3)

    for i, thrust_vec in enumerate(thrust_vectors):
        thrust_vec = np.array(thrust_vec)
        total_force += thrust_vec
        total_torque += np.cross(MODULE_POSITIONS[i], thrust_vec)

    return {
        'force': total_force,
        'torque': total_torque
    }


# ============================================================================
# ROS 發布器
# ============================================================================

class ThrustVectorTestPublisher:
    """從推力向量發布到 RViz"""

    def __init__(self):
        rospy.init_node('test_thrust_vector', anonymous=True)

        # Publishers
        self.servo_pub = rospy.Publisher('px4/actuator_servos', ActuatorServos, queue_size=10)
        self.motor_pub = rospy.Publisher('px4/actuator_motors', ActuatorMotors, queue_size=10)
        self.meta_pub = rospy.Publisher('px4/control_allocation_meta_data', ControlAllocationMetaData, queue_size=10)

        self.rate = rospy.get_param('~rate', 50)

        # 當前推力向量設定 (預設: 懸停)
        hover_per_module = HOVER_THRUST / 4
        self.thrust_vectors = [
            [0, 0, hover_per_module],  # Module 1
            [0, 0, hover_per_module],  # Module 2
            [0, 0, hover_per_module],  # Module 3
            [0, 0, hover_per_module],  # Module 4
        ]

        rospy.loginfo("Thrust Vector Test Publisher initialized")
        rospy.loginfo(f"Hover thrust per module: {hover_per_module:.2f} N")
        rospy.loginfo(f"Max thrust per module: {MAX_THRUST:.2f} N")

    def get_timestamp(self):
        return int(rospy.Time.now().to_sec() * 1e6)

    def set_thrust_vectors(self, vectors: list):
        """
        設定 4 個模組的推力向量

        Args:
            vectors: [[fx1,fy1,fz1], [fx2,fy2,fz2], [fx3,fy3,fz3], [fx4,fy4,fz4]]
        """
        if len(vectors) != 4:
            rospy.logwarn("Must provide exactly 4 thrust vectors")
            return

        self.thrust_vectors = [np.array(v) for v in vectors]
        self._log_current_state()

    def set_module_thrust(self, module: int, fx: float, fy: float, fz: float):
        """
        設定單一模組的推力向量

        Args:
            module: 模組編號 (1-4)
            fx, fy, fz: 推力分量 (N)
        """
        if module < 1 or module > 4:
            rospy.logwarn(f"Invalid module: {module}")
            return

        self.thrust_vectors[module - 1] = np.array([fx, fy, fz])
        self._log_current_state()

    def _log_current_state(self):
        """打印當前狀態"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("Current Thrust Vectors and Computed Values:")
        rospy.loginfo("-" * 60)

        for i, thrust_vec in enumerate(self.thrust_vectors):
            result = thrust_vector_to_joint_angles(thrust_vec)
            rospy.loginfo(f"Module {i+1}:")
            rospy.loginfo(f"  Thrust Vector: [{thrust_vec[0]:.2f}, {thrust_vec[1]:.2f}, {thrust_vec[2]:.2f}] N")
            rospy.loginfo(f"  Thrust Magnitude: {result['thrust_magnitude']:.2f} N")
            rospy.loginfo(f"  eta_x (gimbal_actuator): {np.degrees(result['eta_x']):.2f}°")
            rospy.loginfo(f"  eta_y (body_gimbal): {np.degrees(result['eta_y']):.2f}°")

        wrench = compute_body_wrench(self.thrust_vectors)
        rospy.loginfo("-" * 60)
        rospy.loginfo("Body Wrench:")
        rospy.loginfo(f"  Force:  [{wrench['force'][0]:.2f}, {wrench['force'][1]:.2f}, {wrench['force'][2]:.2f}] N")
        rospy.loginfo(f"  Torque: [{wrench['torque'][0]:.3f}, {wrench['torque'][1]:.3f}, {wrench['torque'][2]:.3f}] N.m")
        rospy.loginfo("=" * 60)

    def publish(self):
        """發布當前設定到 ROS topics"""
        timestamp = self.get_timestamp()

        # 計算各模組的控制值
        servo_controls = [0.0] * 8
        motor_controls = [0.0] * 12

        for i, thrust_vec in enumerate(self.thrust_vectors):
            result = thrust_vector_to_joint_angles(thrust_vec)

            # Servo controls
            servo_controls[2*i] = joint_angle_to_control(result['eta_x'], 'gimbal_actuator')
            servo_controls[2*i + 1] = joint_angle_to_control(result['eta_y'], 'body_gimbal')

            # Motor controls
            motor_controls[i] = thrust_to_motor_control(result['thrust_magnitude'])

        # 發布 servos
        servo_msg = ActuatorServos()
        servo_msg.timestamp = timestamp
        servo_msg.control = servo_controls
        self.servo_pub.publish(servo_msg)

        # 發布 motors
        motor_msg = ActuatorMotors()
        motor_msg.timestamp = timestamp
        motor_msg.control = motor_controls
        self.motor_pub.publish(motor_msg)

        # 計算並發布 wrench
        wrench = compute_body_wrench(self.thrust_vectors)
        meta_msg = ControlAllocationMetaData()
        meta_msg.timestamp = timestamp

        # allocated_control: [tx, ty, tz, fx, fy, fz]
        meta_msg.allocated_control[0] = wrench['torque'][0]
        meta_msg.allocated_control[1] = wrench['torque'][1]
        meta_msg.allocated_control[2] = wrench['torque'][2]
        meta_msg.allocated_control[3] = wrench['force'][0]
        meta_msg.allocated_control[4] = wrench['force'][1]
        meta_msg.allocated_control[5] = wrench['force'][2]

        # control_sp (期望值，設為相同)
        meta_msg.control_sp = meta_msg.allocated_control

        # 偽力 (各模組的推力)
        for i, thrust_vec in enumerate(self.thrust_vectors):
            for j in range(4):  # 4 iterations
                idx = j * 4 + i
                scale = (j + 1) / 4.0
                meta_msg.f_x[idx] = thrust_vec[0] * scale
                meta_msg.f_y[idx] = thrust_vec[1] * scale
                meta_msg.f_z[idx] = thrust_vec[2] * scale

        meta_msg.saturated_idx = [-1, -1, -1, -1]
        meta_msg.increment = [0.25, 0.25, 0.25, 0.25]

        self.meta_pub.publish(meta_msg)

    def run_static(self):
        """靜態模式：持續發布當前設定"""
        rate = rospy.Rate(self.rate)
        self._log_current_state()

        rospy.loginfo("Publishing static thrust vectors to RViz...")

        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def run_demo(self):
        """演示模式：循環展示不同推力配置"""
        rate = rospy.Rate(self.rate)
        start_time = rospy.Time.now()

        hover = HOVER_THRUST / 4

        # 定義測試案例
        test_cases = [
            {
                'name': 'Hover (純懸停)',
                'vectors': [[0, 0, hover], [0, 0, hover], [0, 0, hover], [0, 0, hover]],
                'duration': 3.0
            },
            {
                'name': 'Roll Right (向右傾斜)',
                'vectors': [[2, 0, hover], [-2, 0, hover], [-2, 0, hover], [2, 0, hover]],
                'duration': 3.0
            },
            {
                'name': 'Roll Left (向左傾斜)',
                'vectors': [[-2, 0, hover], [2, 0, hover], [2, 0, hover], [-2, 0, hover]],
                'duration': 3.0
            },
            {
                'name': 'Pitch Forward (向前傾斜)',
                'vectors': [[0, 2, hover], [0, 2, hover], [0, -2, hover], [0, -2, hover]],
                'duration': 3.0
            },
            {
                'name': 'Pitch Backward (向後傾斜)',
                'vectors': [[0, -2, hover], [0, -2, hover], [0, 2, hover], [0, 2, hover]],
                'duration': 3.0
            },
            {
                'name': 'Lateral X+ (向X正方向)',
                'vectors': [[3, 0, hover], [3, 0, hover], [3, 0, hover], [3, 0, hover]],
                'duration': 3.0
            },
            {
                'name': 'Lateral Y+ (向Y正方向)',
                'vectors': [[0, 3, hover], [0, 3, hover], [0, 3, hover], [0, 3, hover]],
                'duration': 3.0
            },
            {
                'name': 'Asymmetric (非對稱)',
                'vectors': [[3, 2, hover*1.2], [-1, 1, hover*0.8], [-2, -1, hover], [1, -2, hover]],
                'duration': 4.0
            },
        ]

        rospy.loginfo("Starting demo mode...")
        rospy.loginfo(f"Total {len(test_cases)} test cases")

        case_idx = 0
        case_start = rospy.Time.now()

        while not rospy.is_shutdown():
            current_case = test_cases[case_idx]
            elapsed = (rospy.Time.now() - case_start).to_sec()

            if elapsed >= current_case['duration']:
                # 切換到下一個案例
                case_idx = (case_idx + 1) % len(test_cases)
                case_start = rospy.Time.now()
                current_case = test_cases[case_idx]

                rospy.loginfo("")
                rospy.loginfo(f">>> Test Case {case_idx + 1}: {current_case['name']}")
                self.set_thrust_vectors(current_case['vectors'])

            self.publish()
            rate.sleep()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Thrust Vector Test for RViz')
    parser.add_argument('--mode', type=str, default='demo',
                       choices=['demo', 'static', 'interactive'],
                       help='demo: cycle through test cases, static: fixed vectors')
    parser.add_argument('--rate', type=int, default=50, help='Publish rate (Hz)')

    args, _ = parser.parse_known_args()

    try:
        publisher = ThrustVectorTestPublisher()

        if args.mode == 'demo':
            publisher.run_demo()
        elif args.mode == 'static':
            publisher.run_static()
        else:
            # Interactive mode - 可以在 Python console 中操作
            rospy.loginfo("Interactive mode. Use publisher.set_module_thrust(module, fx, fy, fz)")
            publisher.run_static()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
