#!/usr/bin/env python3
"""
範例測試腳本：給予各模組推力向量，測試 Joint 與機體 Wrench 的變化

此腳本用於展示：
1. 各模組推力向量如何對應到 joint 角度
2. 如何計算機體的總 wrench (力和力矩)
3. 數據流的完整關係

數據來源說明：
- 實際飛行中，數據來自 PX4 固件的控制分配模組 (Control Allocation)
- 通過 ROS topics 發布：
  - px4/actuator_servos: 伺服角度控制 (8 values, [-1, 1])
  - px4/actuator_motors: 馬達轉速控制 (4 values, [-1, 1])
  - px4/control_allocation_meta_data: 力/力矩分配結果

Author: Auto-generated for TVMD-GZ-Sim
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from mpl_toolkits.mplot3d import Axes3D

# ============================================================================
# 系統配置參數 (來自 URDF 檔案)
# ============================================================================

# 模組數量
NUM_MODULES = 4

# 關節限制 (來自 Single-Agent-Def.xacro)
JOINT_LIMITS = {
    'body_gimbal': {'min': -np.pi/4, 'max': np.pi/4},      # Y軸旋轉, ±45°
    'gimbal_actuator': {'min': -0.349, 'max': 0.349},      # X軸旋轉, ±20°
}

# 馬達參數 (來自 tvmd.gazebo)
MOTOR_CONSTANT = 2e-05        # 推力常數 N/(rad/s)²
MOMENT_CONSTANT = 0.06        # 扭矩常數
MAX_ROT_VELOCITY = 1500       # 最大轉速 rad/s

# 模組位置 (來自 tvmd.xacro，相對於 base_link)
# 格式: [x, y, z] in meters
MODULE_POSITIONS = np.array([
    [0.11, 0.11, 0.0],    # Module 1
    [-0.11, 0.11, 0.0],   # Module 2
    [-0.11, -0.11, 0.0],  # Module 3
    [0.11, -0.11, 0.0],   # Module 4
])

# 機體參數 (來自 Navigator-Def.xacro)
BODY_MASS = 2.83791501  # kg
GRAVITY = 9.81          # m/s²


# ============================================================================
# 核心計算函數
# ============================================================================

def control_to_joint_angle(control_value: float, joint_type: str) -> float:
    """
    將控制值 [-1, 1] 轉換為關節角度 (rad)

    這是 republisher.py 中的映射邏輯：
    joint_angle = control_value * (max - min)/2 + (max + min)/2

    Args:
        control_value: 控制值 [-1, 1]
        joint_type: 'body_gimbal' 或 'gimbal_actuator'

    Returns:
        關節角度 (rad)
    """
    limits = JOINT_LIMITS[joint_type]
    factor = (limits['max'] - limits['min']) / 2
    offset = (limits['max'] + limits['min']) / 2
    return control_value * factor + offset


def motor_control_to_thrust(control_value: float) -> float:
    """
    將馬達控制值 [0, 1] 轉換為推力 (N)

    基於 Gazebo MulticopterMotorModel 插件:
    thrust = motorConstant * ω²

    Args:
        control_value: 馬達控制值 [0, 1]

    Returns:
        推力 (N)
    """
    # 控制值映射到角速度
    omega = control_value * MAX_ROT_VELOCITY
    # 計算推力
    thrust = MOTOR_CONSTANT * omega * omega
    return thrust


def servo_angles_to_thrust_direction(eta_x: float, eta_y: float) -> np.ndarray:
    """
    將伺服角度轉換為推力方向向量 (單位向量)

    Args:
        eta_x: gimbal_actuator 角度 (X軸旋轉)
        eta_y: body_gimbal 角度 (Y軸旋轉)

    Returns:
        推力方向單位向量 [dx, dy, dz]
    """
    # 旋轉矩陣組合 (先繞Y軸，再繞X軸)
    Ry = np.array([
        [np.cos(eta_y), 0, np.sin(eta_y)],
        [0, 1, 0],
        [-np.sin(eta_y), 0, np.cos(eta_y)]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(eta_x), -np.sin(eta_x)],
        [0, np.sin(eta_x), np.cos(eta_x)]
    ])

    # 初始推力方向為 +Z (向上)
    initial_direction = np.array([0, 0, 1])

    # 應用旋轉
    direction = Ry @ Rx @ initial_direction
    return direction / np.linalg.norm(direction)


def compute_module_thrust_vector(motor_ctrl: float, servo_x_ctrl: float, servo_y_ctrl: float) -> tuple:
    """
    計算單一模組的推力向量

    Args:
        motor_ctrl: 馬達控制值 [0, 1]
        servo_x_ctrl: X軸伺服控制值 [-1, 1]
        servo_y_ctrl: Y軸伺服控制值 [-1, 1]

    Returns:
        (thrust_magnitude, thrust_direction, joint_angles)
    """
    # 計算推力大小
    thrust_mag = motor_control_to_thrust(motor_ctrl)

    # 計算關節角度
    eta_x = control_to_joint_angle(servo_x_ctrl, 'gimbal_actuator')
    eta_y = control_to_joint_angle(servo_y_ctrl, 'body_gimbal')

    # 計算推力方向
    thrust_dir = servo_angles_to_thrust_direction(eta_x, eta_y)

    return thrust_mag, thrust_dir, {'eta_x': eta_x, 'eta_y': eta_y}


def compute_body_wrench(module_thrusts: list, module_directions: list) -> dict:
    """
    計算機體的總 wrench (力和力矩)

    Args:
        module_thrusts: 各模組的推力大小 [N]
        module_directions: 各模組的推力方向 (單位向量)

    Returns:
        {'force': [fx, fy, fz], 'torque': [tx, ty, tz]}
    """
    total_force = np.zeros(3)
    total_torque = np.zeros(3)

    for i in range(NUM_MODULES):
        # 推力向量
        thrust_vec = module_thrusts[i] * module_directions[i]
        total_force += thrust_vec

        # 力矩 = 位置 × 推力
        torque = np.cross(MODULE_POSITIONS[i], thrust_vec)
        total_torque += torque

    return {'force': total_force, 'torque': total_torque}


# ============================================================================
# 測試範例
# ============================================================================

def generate_test_data(duration: float = 10.0, dt: float = 0.01) -> dict:
    """
    生成測試數據：模擬各種推力向量配置

    Args:
        duration: 測試持續時間 (秒)
        dt: 時間步長 (秒)

    Returns:
        包含所有數據的字典
    """
    t = np.arange(0, duration, dt)
    n_steps = len(t)

    # 初始化數據存儲
    data = {
        'time': t,
        'motor_controls': np.zeros((n_steps, NUM_MODULES)),
        'servo_x_controls': np.zeros((n_steps, NUM_MODULES)),
        'servo_y_controls': np.zeros((n_steps, NUM_MODULES)),
        'joint_eta_x': np.zeros((n_steps, NUM_MODULES)),
        'joint_eta_y': np.zeros((n_steps, NUM_MODULES)),
        'thrust_magnitudes': np.zeros((n_steps, NUM_MODULES)),
        'thrust_vectors': np.zeros((n_steps, NUM_MODULES, 3)),
        'body_force': np.zeros((n_steps, 3)),
        'body_torque': np.zeros((n_steps, 3)),
    }

    # 懸停所需的基礎推力 (平均分配重力)
    hover_thrust_per_module = (BODY_MASS * GRAVITY) / NUM_MODULES
    hover_motor_ctrl = np.sqrt(hover_thrust_per_module / MOTOR_CONSTANT) / MAX_ROT_VELOCITY

    print(f"懸停推力 (每模組): {hover_thrust_per_module:.2f} N")
    print(f"懸停馬達控制值: {hover_motor_ctrl:.3f}")

    for step, time in enumerate(t):
        # === 定義測試控制輸入 ===

        # 階段 1 (0-2s): 純懸停
        if time < 2:
            motor_ctrls = [hover_motor_ctrl] * 4
            servo_x_ctrls = [0, 0, 0, 0]
            servo_y_ctrls = [0, 0, 0, 0]

        # 階段 2 (2-4s): Roll 控制 (差異化 X 軸伺服)
        elif time < 4:
            motor_ctrls = [hover_motor_ctrl] * 4
            # 模組 1,4 向右傾斜，模組 2,3 向左傾斜
            tilt = 0.3 * np.sin(2 * np.pi * (time - 2))
            servo_x_ctrls = [tilt, -tilt, -tilt, tilt]
            servo_y_ctrls = [0, 0, 0, 0]

        # 階段 3 (4-6s): Pitch 控制 (差異化 Y 軸伺服)
        elif time < 6:
            motor_ctrls = [hover_motor_ctrl] * 4
            servo_x_ctrls = [0, 0, 0, 0]
            # 模組 1,2 向前傾斜，模組 3,4 向後傾斜
            tilt = 0.3 * np.sin(2 * np.pi * (time - 4))
            servo_y_ctrls = [tilt, tilt, -tilt, -tilt]

        # 階段 4 (6-8s): Yaw 控制 (差異化馬達轉速)
        elif time < 8:
            delta = 0.05 * np.sin(2 * np.pi * (time - 6))
            motor_ctrls = [hover_motor_ctrl + delta,
                          hover_motor_ctrl - delta,
                          hover_motor_ctrl + delta,
                          hover_motor_ctrl - delta]
            servo_x_ctrls = [0, 0, 0, 0]
            servo_y_ctrls = [0, 0, 0, 0]

        # 階段 5 (8-10s): 組合控制
        else:
            motor_ctrls = [hover_motor_ctrl * 1.1] * 4  # 略微增加推力
            tilt_x = 0.2 * np.sin(2 * np.pi * (time - 8))
            tilt_y = 0.2 * np.cos(2 * np.pi * (time - 8))
            servo_x_ctrls = [tilt_x, -tilt_x, -tilt_x, tilt_x]
            servo_y_ctrls = [tilt_y, tilt_y, -tilt_y, -tilt_y]

        # === 計算各模組的推力 ===
        thrust_mags = []
        thrust_dirs = []

        for i in range(NUM_MODULES):
            thrust_mag, thrust_dir, joints = compute_module_thrust_vector(
                motor_ctrls[i], servo_x_ctrls[i], servo_y_ctrls[i]
            )
            thrust_mags.append(thrust_mag)
            thrust_dirs.append(thrust_dir)

            # 存儲數據
            data['motor_controls'][step, i] = motor_ctrls[i]
            data['servo_x_controls'][step, i] = servo_x_ctrls[i]
            data['servo_y_controls'][step, i] = servo_y_ctrls[i]
            data['joint_eta_x'][step, i] = joints['eta_x']
            data['joint_eta_y'][step, i] = joints['eta_y']
            data['thrust_magnitudes'][step, i] = thrust_mag
            data['thrust_vectors'][step, i] = thrust_mag * thrust_dir

        # === 計算機體 wrench ===
        wrench = compute_body_wrench(thrust_mags, thrust_dirs)
        data['body_force'][step] = wrench['force']
        data['body_torque'][step] = wrench['torque']

    return data


def plot_test_results(data: dict, output_folder: str = None):
    """
    繪製測試結果
    """
    t = data['time']

    # Figure 1: 控制輸入
    fig1, axes1 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig1.suptitle('Control Inputs (ActuatorMotors & ActuatorServos)', fontsize=14)

    colors = ['#0072BD', '#D95319', '#EDB120', '#7E2F8E']

    for i in range(NUM_MODULES):
        axes1[0].plot(t, data['motor_controls'][:, i], color=colors[i],
                     label=f'Module {i+1}', linewidth=1.5)
    axes1[0].set_ylabel('Motor Control [-1,1]')
    axes1[0].legend(loc='upper right', ncol=4)
    axes1[0].grid(True, alpha=0.3)

    for i in range(NUM_MODULES):
        axes1[1].plot(t, data['servo_x_controls'][:, i], color=colors[i], linewidth=1.5)
    axes1[1].set_ylabel('Servo X (gimbal_actuator) [-1,1]')
    axes1[1].grid(True, alpha=0.3)

    for i in range(NUM_MODULES):
        axes1[2].plot(t, data['servo_y_controls'][:, i], color=colors[i], linewidth=1.5)
    axes1[2].set_ylabel('Servo Y (body_gimbal) [-1,1]')
    axes1[2].set_xlabel('Time [s]')
    axes1[2].grid(True, alpha=0.3)

    plt.tight_layout()

    # Figure 2: 關節角度
    fig2, axes2 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig2.suptitle('Joint Angles (Computed from Control Inputs)', fontsize=14)

    for i in range(NUM_MODULES):
        axes2[0].plot(t, np.degrees(data['joint_eta_x'][:, i]), color=colors[i],
                     label=f'Module {i+1}', linewidth=1.5)
    axes2[0].set_ylabel(r'$\eta_x$ (gimbal_actuator) [deg]')
    axes2[0].axhline(y=20, color='r', linestyle='--', alpha=0.5, label='Limit')
    axes2[0].axhline(y=-20, color='r', linestyle='--', alpha=0.5)
    axes2[0].legend(loc='upper right', ncol=5)
    axes2[0].grid(True, alpha=0.3)

    for i in range(NUM_MODULES):
        axes2[1].plot(t, np.degrees(data['joint_eta_y'][:, i]), color=colors[i], linewidth=1.5)
    axes2[1].set_ylabel(r'$\eta_y$ (body_gimbal) [deg]')
    axes2[1].axhline(y=45, color='r', linestyle='--', alpha=0.5, label='Limit')
    axes2[1].axhline(y=-45, color='r', linestyle='--', alpha=0.5)
    axes2[1].set_xlabel('Time [s]')
    axes2[1].grid(True, alpha=0.3)

    plt.tight_layout()

    # Figure 3: 推力大小
    fig3, axes3 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig3.suptitle('Thrust Magnitudes per Module', fontsize=14)

    for i in range(NUM_MODULES):
        axes3[0].plot(t, data['thrust_magnitudes'][:, i], color=colors[i],
                     label=f'Module {i+1}', linewidth=1.5)
    axes3[0].set_ylabel('Thrust [N]')
    axes3[0].axhline(y=BODY_MASS*GRAVITY/4, color='gray', linestyle='--',
                    alpha=0.5, label='Hover thrust')
    axes3[0].legend(loc='upper right', ncol=5)
    axes3[0].grid(True, alpha=0.3)

    axes3[1].plot(t, np.sum(data['thrust_magnitudes'], axis=1), 'k-', linewidth=2)
    axes3[1].axhline(y=BODY_MASS*GRAVITY, color='r', linestyle='--',
                    alpha=0.5, label='Weight')
    axes3[1].set_ylabel('Total Thrust [N]')
    axes3[1].set_xlabel('Time [s]')
    axes3[1].legend(loc='upper right')
    axes3[1].grid(True, alpha=0.3)

    plt.tight_layout()

    # Figure 4: 機體 Wrench
    fig4, axes4 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig4.suptitle('Body Wrench (Force & Torque)', fontsize=14)

    force_colors = ['#0072BD', '#D95319', '#EDB120']
    axes4[0].plot(t, data['body_force'][:, 0], color=force_colors[0],
                 label=r'$F_x$', linewidth=1.5)
    axes4[0].plot(t, data['body_force'][:, 1], color=force_colors[1],
                 label=r'$F_y$', linewidth=1.5)
    axes4[0].plot(t, data['body_force'][:, 2], color=force_colors[2],
                 label=r'$F_z$', linewidth=1.5)
    axes4[0].axhline(y=BODY_MASS*GRAVITY, color='gray', linestyle='--',
                    alpha=0.5, label='Weight')
    axes4[0].set_ylabel('Force [N]')
    axes4[0].legend(loc='upper right', ncol=4)
    axes4[0].grid(True, alpha=0.3)

    axes4[1].plot(t, data['body_torque'][:, 0], color=force_colors[0],
                 label=r'$\tau_x$ (Roll)', linewidth=1.5)
    axes4[1].plot(t, data['body_torque'][:, 1], color=force_colors[1],
                 label=r'$\tau_y$ (Pitch)', linewidth=1.5)
    axes4[1].plot(t, data['body_torque'][:, 2], color=force_colors[2],
                 label=r'$\tau_z$ (Yaw)', linewidth=1.5)
    axes4[1].set_ylabel('Torque [N.m]')
    axes4[1].set_xlabel('Time [s]')
    axes4[1].legend(loc='upper right', ncol=3)
    axes4[1].grid(True, alpha=0.3)

    plt.tight_layout()

    # Figure 5: 3D 推力向量可視化 (特定時間點)
    fig5 = plt.figure(figsize=(12, 5))

    time_points = [1.0, 3.0, 5.0, 9.0]  # 各階段的代表時間點
    titles = ['Hover', 'Roll Control', 'Pitch Control', 'Combined']

    for idx, (time_pt, title) in enumerate(zip(time_points, titles)):
        ax = fig5.add_subplot(1, 4, idx+1, projection='3d')
        step = int(time_pt / (t[1] - t[0]))

        # 繪製模組位置和推力向量
        for i in range(NUM_MODULES):
            pos = MODULE_POSITIONS[i]
            thrust_vec = data['thrust_vectors'][step, i] * 0.05  # 縮放以便顯示

            # 繪製模組位置
            ax.scatter(*pos, s=50, c=colors[i], marker='o')

            # 繪製推力向量
            ax.quiver(pos[0], pos[1], pos[2],
                     thrust_vec[0], thrust_vec[1], thrust_vec[2],
                     color=colors[i], arrow_length_ratio=0.2, linewidth=2)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'{title}\nt={time_pt}s')
        ax.set_xlim([-0.2, 0.2])
        ax.set_ylim([-0.2, 0.2])
        ax.set_zlim([0, 0.4])

    plt.tight_layout()

    if output_folder:
        for i, fig in enumerate([fig1, fig2, fig3, fig4, fig5]):
            fig.savefig(f'{output_folder}/test_result_{i+1}.png', dpi=150, bbox_inches='tight')

    plt.show()


def print_data_sources_info():
    """
    列印數據來源說明
    """
    info = """
    ============================================================================
    TVMD-GZ-Sim 數據來源說明
    ============================================================================

    【數據流架構】

    PX4 固件 (Control Allocation)
         │
         ├── ActuatorServos (8 values) ──→ 伺服角度
         │     └── Topic: px4/actuator_servos
         │     └── 控制值範圍: [-1, 1]
         │     └── 映射: control[0,2,4,6] → gimbal_actuator (X軸)
         │              control[1,3,5,7] → body_gimbal (Y軸)
         │
         ├── ActuatorMotors (4 values) ──→ 馬達轉速
         │     └── Topic: px4/actuator_motors
         │     └── 控制值範圍: [0, 1]
         │     └── 映射: control[0-3] → 模組 1-4 的螺旋槳
         │
         └── ControlAllocationMetaData ──→ Wrench
               └── Topic: px4/control_allocation_meta_data
               └── control_sp[0:3]: 期望扭矩 [τx, τy, τz]
               └── control_sp[3:6]: 期望推力 [fx, fy, fz]
               └── allocated_control[0:6]: 實際分配結果

    【關節角度映射公式】

    joint_angle = control_value × (max - min)/2 + (max + min)/2

    - gimbal_actuator: angle = control × 0.349 (±20°)
    - body_gimbal: angle = control × π/4 (±45°)

    【推力計算公式】

    thrust = motorConstant × ω²

    - motorConstant = 2e-05 N/(rad/s)²
    - ω = control_value × maxRotVelocity
    - maxRotVelocity = 1500 rad/s

    【機體 Wrench 計算】

    Total Force = Σ (thrust_i × direction_i)
    Total Torque = Σ (position_i × thrust_vector_i)

    【配置檔案位置】

    - 關節定義: src/tvmd/urdf/Single-Agent-Def.xacro
    - 馬達參數: src/tvmd/urdf/tvmd.gazebo
    - 機體參數: src/tvmd/urdf/Navigator-Def.xacro
    - 數據轉換: src/tvmd/republisher.py

    【日誌檔案格式】

    - 格式: ULog (.ulg) - PX4 原生二進制格式
    - 讀取: 使用 pyulog 庫
    - 工具: src/tvmd/Visualizer.py

    ============================================================================
    """
    print(info)


# ============================================================================
# 主程式
# ============================================================================

if __name__ == '__main__':
    print_data_sources_info()

    print("\n生成測試數據...")
    data = generate_test_data(duration=10.0, dt=0.01)

    print("\n測試階段說明:")
    print("  0-2s: 純懸停 (所有伺服歸零)")
    print("  2-4s: Roll 控制 (X軸伺服差異化)")
    print("  4-6s: Pitch 控制 (Y軸伺服差異化)")
    print("  6-8s: Yaw 控制 (馬達轉速差異化)")
    print("  8-10s: 組合控制")

    print("\n繪製結果...")
    plot_test_results(data)
