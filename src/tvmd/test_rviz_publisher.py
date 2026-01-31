#!/usr/bin/env python3
"""
RViz 測試發布器：發布合成數據到 PX4 topics，讓 RViz 顯示 joint 和 wrench 的變化

使用方式:
1. 啟動 display.launch: roslaunch tvmd display.launch
2. 執行此腳本: rosrun tvmd test_rviz_publisher.py

或使用專用的測試 launch 文件:
   roslaunch tvmd test_rviz.launch

此腳本會發布以下 topics:
- px4/actuator_servos: 伺服控制 → 關節角度
- px4/actuator_motors: 馬達控制 → 螺旋槳旋轉
- px4/control_allocation_meta_data: 控制分配 → Wrench 箭頭
- px4/vehicle_attitude: 姿態 → 機體旋轉

Author: Auto-generated for TVMD-GZ-Sim
"""

import rospy
import numpy as np
import math

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import ControlAllocationMetaData
from px4_msgs.msg import VehicleAttitude


class TestDataPublisher:
    """發布測試數據到 PX4 topics"""

    def __init__(self):
        rospy.init_node('test_rviz_publisher', anonymous=True)

        # Publishers
        self.servo_pub = rospy.Publisher('px4/actuator_servos', ActuatorServos, queue_size=10)
        self.motor_pub = rospy.Publisher('px4/actuator_motors', ActuatorMotors, queue_size=10)
        self.meta_pub = rospy.Publisher('px4/control_allocation_meta_data', ControlAllocationMetaData, queue_size=10)
        self.attitude_pub = rospy.Publisher('px4/vehicle_attitude', VehicleAttitude, queue_size=10)

        # 參數
        self.rate = rospy.get_param('~rate', 50)  # Hz
        self.test_mode = rospy.get_param('~test_mode', 'all')  # 'servo', 'motor', 'wrench', 'attitude', 'all'

        # 狀態
        self.start_time = None
        self.prop_angles = [0.0, 0.0, 0.0, 0.0]  # 累積螺旋槳角度

        rospy.loginfo(f"Test publisher initialized. Mode: {self.test_mode}, Rate: {self.rate} Hz")

    def get_timestamp(self):
        """獲取微秒級時間戳"""
        return int(rospy.Time.now().to_sec() * 1e6)

    def publish_servos(self, t: float, servo_values: list):
        """
        發布伺服控制訊息

        Args:
            t: 時間 (秒)
            servo_values: 8個伺服控制值 [-1, 1]
                [0]: Module 1 gimbal_actuator (X)
                [1]: Module 1 body_gimbal (Y)
                [2]: Module 2 gimbal_actuator (X)
                ...
        """
        msg = ActuatorServos()
        msg.timestamp = self.get_timestamp()
        msg.control = [0.0] * 8
        for i in range(min(8, len(servo_values))):
            msg.control[i] = np.clip(servo_values[i], -1.0, 1.0)
        self.servo_pub.publish(msg)

    def publish_motors(self, t: float, motor_values: list):
        """
        發布馬達控制訊息

        Args:
            t: 時間 (秒)
            motor_values: 4個馬達控制值 [0, 1]
        """
        msg = ActuatorMotors()
        msg.timestamp = self.get_timestamp()
        msg.control = [0.0] * 12
        for i in range(min(4, len(motor_values))):
            msg.control[i] = np.clip(motor_values[i], 0.0, 1.0)
        self.motor_pub.publish(msg)

    def publish_control_allocation(self, t: float, desired_wrench: list, allocated_wrench: list,
                                    pseudo_forces: dict = None):
        """
        發布控制分配訊息

        Args:
            t: 時間 (秒)
            desired_wrench: 期望 wrench [tx, ty, tz, fx, fy, fz]
            allocated_wrench: 實際分配 wrench [tx, ty, tz, fx, fy, fz]
            pseudo_forces: 偽力數據 (可選)
        """
        msg = ControlAllocationMetaData()
        msg.timestamp = self.get_timestamp()

        # 設定 control setpoint 和 allocated control
        for i in range(6):
            msg.control_sp[i] = desired_wrench[i] if i < len(desired_wrench) else 0.0
            msg.allocated_control[i] = allocated_wrench[i] if i < len(allocated_wrench) else 0.0

        # 設定偽力 (用於可視化迭代過程)
        if pseudo_forces:
            for i in range(16):
                msg.f_x[i] = pseudo_forces.get('f_x', [0]*16)[i]
                msg.f_y[i] = pseudo_forces.get('f_y', [0]*16)[i]
                msg.f_z[i] = pseudo_forces.get('f_z', [0]*16)[i]
        else:
            # 默認：設置一些簡單的偽力數據
            for i in range(4):  # 4個模組
                for j in range(4):  # 4次迭代
                    idx = j * 4 + i
                    scale = (j + 1) / 4.0
                    msg.f_x[idx] = allocated_wrench[3] * scale / 4.0  # fx
                    msg.f_y[idx] = allocated_wrench[4] * scale / 4.0  # fy
                    msg.f_z[idx] = allocated_wrench[5] * scale / 4.0  # fz

        # 飽和指標
        msg.saturated_idx = [-1, -1, -1, -1]
        msg.increment = [0.25, 0.25, 0.25, 0.25]

        self.meta_pub.publish(msg)

    def publish_attitude(self, t: float, roll: float, pitch: float, yaw: float):
        """
        發布姿態訊息

        Args:
            t: 時間 (秒)
            roll, pitch, yaw: 歐拉角 (rad)
        """
        msg = VehicleAttitude()
        msg.timestamp = self.get_timestamp()

        # 歐拉角轉四元數 (wxyz 格式)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        msg.q[0] = cr * cp * cy + sr * sp * sy  # w
        msg.q[1] = sr * cp * cy - cr * sp * sy  # x
        msg.q[2] = cr * sp * cy + sr * cp * sy  # y
        msg.q[3] = cr * cp * sy - sr * sp * cy  # z

        msg.delta_q_reset = [1.0, 0.0, 0.0, 0.0]  # 無重置

        self.attitude_pub.publish(msg)

    def generate_test_scenario(self, t: float) -> dict:
        """
        生成測試場景數據

        Args:
            t: 時間 (秒)

        Returns:
            包含所有控制數據的字典
        """
        scenario = {
            'servos': [0.0] * 8,
            'motors': [0.5] * 4,  # 基礎懸停推力
            'desired_wrench': [0.0, 0.0, 0.0, 0.0, 0.0, 27.8],  # 懸停力
            'allocated_wrench': [0.0, 0.0, 0.0, 0.0, 0.0, 27.8],
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
        }

        # 測試階段 (每個階段 4 秒)
        phase = int(t / 4) % 6
        phase_t = t % 4  # 階段內時間

        if phase == 0:
            # 階段 0: 純懸停
            rospy.loginfo_throttle(4, "Phase 0: Hover (no servo movement)")

        elif phase == 1:
            # 階段 1: Roll 控制 - X軸伺服差異化
            rospy.loginfo_throttle(4, "Phase 1: Roll control (X-axis servo)")
            tilt = 0.5 * np.sin(2 * np.pi * phase_t / 2)
            # Module 1,4 向右，Module 2,3 向左
            scenario['servos'][0] = tilt   # M1 X
            scenario['servos'][2] = -tilt  # M2 X
            scenario['servos'][4] = -tilt  # M3 X
            scenario['servos'][6] = tilt   # M4 X
            # 對應的 wrench
            scenario['desired_wrench'][0] = tilt * 2  # tx (roll torque)
            scenario['allocated_wrench'][0] = tilt * 2
            scenario['roll'] = tilt * 0.2

        elif phase == 2:
            # 階段 2: Pitch 控制 - Y軸伺服差異化
            rospy.loginfo_throttle(4, "Phase 2: Pitch control (Y-axis servo)")
            tilt = 0.5 * np.sin(2 * np.pi * phase_t / 2)
            # Module 1,2 向前，Module 3,4 向後
            scenario['servos'][1] = tilt   # M1 Y
            scenario['servos'][3] = tilt   # M2 Y
            scenario['servos'][5] = -tilt  # M3 Y
            scenario['servos'][7] = -tilt  # M4 Y
            # 對應的 wrench
            scenario['desired_wrench'][1] = tilt * 2  # ty (pitch torque)
            scenario['allocated_wrench'][1] = tilt * 2
            scenario['pitch'] = tilt * 0.2

        elif phase == 3:
            # 階段 3: Yaw 控制 - 馬達轉速差異化
            rospy.loginfo_throttle(4, "Phase 3: Yaw control (motor speed)")
            delta = 0.1 * np.sin(2 * np.pi * phase_t / 2)
            scenario['motors'] = [0.5 + delta, 0.5 - delta, 0.5 + delta, 0.5 - delta]
            # 對應的 wrench
            scenario['desired_wrench'][2] = delta * 5  # tz (yaw torque)
            scenario['allocated_wrench'][2] = delta * 5
            scenario['yaw'] = delta * 0.5

        elif phase == 4:
            # 階段 4: 水平移動 - 所有伺服同向傾斜
            rospy.loginfo_throttle(4, "Phase 4: Lateral force (all servos tilt)")
            tilt_x = 0.3 * np.sin(2 * np.pi * phase_t / 2)
            tilt_y = 0.3 * np.cos(2 * np.pi * phase_t / 2)
            for i in range(4):
                scenario['servos'][2*i] = tilt_x    # X
                scenario['servos'][2*i + 1] = tilt_y  # Y
            # 對應的 wrench
            scenario['desired_wrench'][3] = tilt_x * 10  # fx
            scenario['desired_wrench'][4] = tilt_y * 10  # fy
            scenario['allocated_wrench'][3] = tilt_x * 10
            scenario['allocated_wrench'][4] = tilt_y * 10

        elif phase == 5:
            # 階段 5: 組合控制
            rospy.loginfo_throttle(4, "Phase 5: Combined control")
            # 伺服
            tilt = 0.3 * np.sin(2 * np.pi * phase_t / 3)
            scenario['servos'] = [
                tilt, tilt,      # M1
                -tilt, tilt,     # M2
                -tilt, -tilt,    # M3
                tilt, -tilt,     # M4
            ]
            # 馬達
            scenario['motors'] = [0.5 + 0.1 * np.sin(phase_t),
                                   0.5 + 0.1 * np.cos(phase_t),
                                   0.5 - 0.1 * np.sin(phase_t),
                                   0.5 - 0.1 * np.cos(phase_t)]
            # Wrench
            scenario['desired_wrench'] = [tilt, tilt * 0.5, 0.2 * np.sin(phase_t),
                                           tilt * 5, tilt * 3, 28]
            scenario['allocated_wrench'] = scenario['desired_wrench'].copy()
            # 姿態
            scenario['roll'] = tilt * 0.1
            scenario['pitch'] = tilt * 0.1 * np.cos(phase_t)
            scenario['yaw'] = 0.1 * np.sin(phase_t)

        return scenario

    def run(self):
        """主循環"""
        rate = rospy.Rate(self.rate)
        self.start_time = rospy.Time.now()

        rospy.loginfo("Starting test data publication...")
        rospy.loginfo("Test phases (4s each):")
        rospy.loginfo("  0: Hover (baseline)")
        rospy.loginfo("  1: Roll control (X-axis servo)")
        rospy.loginfo("  2: Pitch control (Y-axis servo)")
        rospy.loginfo("  3: Yaw control (motor speed)")
        rospy.loginfo("  4: Lateral force (all servos)")
        rospy.loginfo("  5: Combined control")

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - self.start_time).to_sec()

            # 生成測試數據
            scenario = self.generate_test_scenario(t)

            # 發布數據
            timestamp = self.get_timestamp()

            if self.test_mode in ['servo', 'all']:
                self.publish_servos(t, scenario['servos'])

            if self.test_mode in ['motor', 'all']:
                self.publish_motors(t, scenario['motors'])

            if self.test_mode in ['wrench', 'all']:
                self.publish_control_allocation(
                    t,
                    scenario['desired_wrench'],
                    scenario['allocated_wrench']
                )

            if self.test_mode in ['attitude', 'all']:
                self.publish_attitude(
                    t,
                    scenario['roll'],
                    scenario['pitch'],
                    scenario['yaw']
                )

            rate.sleep()


class InteractiveTestPublisher(TestDataPublisher):
    """互動式測試發布器 - 可以手動設定控制值"""

    def __init__(self):
        super().__init__()

        # 當前控制值
        self.current_servos = [0.0] * 8
        self.current_motors = [0.5] * 4
        self.current_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 27.8]

        # 訂閱控制指令 (可選)
        # rospy.Subscriber('/test_control/servos', Float32MultiArray, self.servo_callback)

        rospy.loginfo("Interactive test publisher initialized.")
        rospy.loginfo("Use set_* methods to change control values.")

    def set_servo(self, module: int, axis: str, value: float):
        """
        設定單一伺服控制值

        Args:
            module: 模組編號 (1-4)
            axis: 'x' (gimbal_actuator) 或 'y' (body_gimbal)
            value: 控制值 [-1, 1]
        """
        if module < 1 or module > 4:
            rospy.logwarn(f"Invalid module: {module}. Must be 1-4.")
            return

        idx = (module - 1) * 2 + (0 if axis.lower() == 'x' else 1)
        self.current_servos[idx] = np.clip(value, -1.0, 1.0)
        rospy.loginfo(f"Servo M{module}{axis.upper()} = {value:.2f}")

    def set_motor(self, module: int, value: float):
        """
        設定單一馬達控制值

        Args:
            module: 模組編號 (1-4)
            value: 控制值 [0, 1]
        """
        if module < 1 or module > 4:
            rospy.logwarn(f"Invalid module: {module}. Must be 1-4.")
            return

        self.current_motors[module - 1] = np.clip(value, 0.0, 1.0)
        rospy.loginfo(f"Motor M{module} = {value:.2f}")

    def set_all_servos(self, values: list):
        """設定所有伺服控制值"""
        for i in range(min(8, len(values))):
            self.current_servos[i] = np.clip(values[i], -1.0, 1.0)
        rospy.loginfo(f"All servos set: {self.current_servos}")

    def set_all_motors(self, values: list):
        """設定所有馬達控制值"""
        for i in range(min(4, len(values))):
            self.current_motors[i] = np.clip(values[i], 0.0, 1.0)
        rospy.loginfo(f"All motors set: {self.current_motors}")

    def run_interactive(self):
        """互動模式主循環"""
        rate = rospy.Rate(self.rate)
        self.start_time = rospy.Time.now()

        rospy.loginfo("Running in interactive mode. Publishing current control values...")

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - self.start_time).to_sec()

            self.publish_servos(t, self.current_servos)
            self.publish_motors(t, self.current_motors)
            self.publish_control_allocation(t, self.current_wrench, self.current_wrench)
            self.publish_attitude(t, 0, 0, 0)

            rate.sleep()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='RViz Test Publisher')
    parser.add_argument('--mode', type=str, default='auto',
                       choices=['auto', 'interactive'],
                       help='Test mode: auto (automatic scenarios) or interactive')
    parser.add_argument('--rate', type=int, default=50,
                       help='Publish rate in Hz')

    # ROS 參數覆蓋命令行參數
    args, _ = parser.parse_known_args()

    try:
        if args.mode == 'interactive':
            publisher = InteractiveTestPublisher()
            publisher.run_interactive()
        else:
            publisher = TestDataPublisher()
            publisher.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
