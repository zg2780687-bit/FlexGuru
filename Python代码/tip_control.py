"""
SO-ARM101 尖端键盘控制

键盘
 -> 尖端 XYZ 位移
 -> 直线轨迹
 -> IK
 -> 5个关节角
 -> RAW
 -> 舵机

6号夹爪保持当前 RAW。
IK 无解时停在最后一个有解的位置。
"""

import time
from pathlib import Path

import numpy as np

from config import CALIBRATION_FILE, JOINTS, JOINT_SPEEDS
from robot_bus import (
    create_bus,
    close_bus,
    read_all_raw,
    set_torque,
)

from so_arm_kinematics import SOArmKinematics


STEP_SIZE = 0.005          # 每次按键移动 5 mm
TRAJECTORY_STEPS = 10      # 一次移动拆成 10 个点
TRAJECTORY_DELAY = 0.05    # 每个点之间等待 50 ms


KEY_DIRECTION = {
    "w": np.array([+1.0, 0.0, 0.0]),  # 前
    "s": np.array([-1.0, 0.0, 0.0]),  # 后
    "a": np.array([0.0, +1.0, 0.0]),  # 左
    "d": np.array([0.0, -1.0, 0.0]),  # 右
    "r": np.array([0.0, 0.0, +1.0]),  # 上
    "f": np.array([0.0, 0.0, -1.0]),  # 下
}


def send_joint_target(bus, target_raw):
    """发送5个关节目标，夹爪也会带着当前 RAW 一起发送。"""
    bus.sync_write(
        "Goal_Velocity",
        JOINT_SPEEDS,
        normalize=False,
    )

    bus.sync_write(
        "Goal_Position",
        target_raw,
        normalize=False,
    )


def print_raw(raw):
    print("\n当前 RAW：")
    for joint in JOINTS:
        print(f"  {joint:<16} {raw[joint]}")


def print_angles(angles):
    print("\n当前关节角：")
    for joint, angle in angles.items():
        print(f"  {joint:<16} {angle:7.2f}°")


def print_position(pose):
    p = pose[:3, 3]
    print(
        f"\n当前尖端位置："
        f" X={p[0]:.4f} m"
        f"  Y={p[1]:.4f} m"
        f"  Z={p[2]:.4f} m"
    )


def move_cartesian(bus, kin, current_raw, current_pose, delta):
    """
    沿一条直线移动一次。

    如果某个轨迹点 IK 无解：
    立即停止，并返回最后一个成功的位置。
    下一次按键从这个位置继续。
    """
    last_raw = dict(current_raw)
    last_pose = current_pose.copy()

    for step in range(1, TRAJECTORY_STEPS + 1):

        target_pose = kin.interpolate_pose(
            current_pose,
            delta,
            step,
            TRAJECTORY_STEPS,
        )

        current_angles = kin.raw_to_joint_angles(last_raw)

        solution = kin.inverse(
            current_angles,
            target_pose,
        )

        if solution is None:
            print(
                f"\nIK 无解：第 {step}/{TRAJECTORY_STEPS} 个轨迹点"
            )
            print("停止在最后一个有解的位置。")
            return last_raw, last_pose

        target_raw = kin.joint_angles_to_raw(
            solution,
            last_raw,
        )

        if not kin.check_raw_safe(target_raw):
            print("\n目标 RAW 超出安全标定范围。")
            print("停止在最后一个有解的位置。")
            return last_raw, last_pose

        send_joint_target(bus, target_raw)

        last_raw = target_raw
        last_pose = target_pose

        progress = delta * step / TRAJECTORY_STEPS * 1000
        print(
            f"  轨迹点 {step}/{TRAJECTORY_STEPS}"
            f"  ΔXYZ = {progress} mm"
        )

        time.sleep(TRAJECTORY_DELAY)

    return last_raw, last_pose


def main():

    print("========================================")
    print("        SO-ARM101 尖端键盘控制")
    print("========================================")
    print("W → 前    S → 后")
    print("A → 左    D → 右")
    print("R → 上    F → 下")
    print("Q → 退出")
    print(f"每次移动：{STEP_SIZE * 1000:.0f} mm")
    print("========================================")

    # calibration.json 在 Python代码目录中
    calibration_path = Path(CALIBRATION_FILE)

    kin = SOArmKinematics(
        calibration_path=calibration_path
    )

    print(f"\nURDF：{kin.urdf_path}")
    print("FK / IK / RAW转换已加载。")

    bus = create_bus()
    torque_enabled = False

    try:
        bus.connect(handshake=False)

        current_raw = read_all_raw(bus)
        print_raw(current_raw)

        if not kin.check_raw_safe(current_raw):
            print("\n当前位置已经超出标定安全范围。")
            return

        current_angles = kin.raw_to_joint_angles(current_raw)
        print_angles(current_angles)

        current_pose = kin.forward(current_angles)
        print_position(current_pose)

        failures = set_torque(bus, True)

        if failures:
            print("\n舵机上力失败。")
            return

        torque_enabled = True
        print("\n进入尖端控制模式。")

        while True:

            key = input(
                "\n请输入 W/A/S/D/R/F/Q："
            ).strip().lower()

            if key == "q":
                print("\n退出尖端控制。")
                break

            if key not in KEY_DIRECTION:
                print("无效按键。")
                continue

            delta = KEY_DIRECTION[key] * STEP_SIZE

            current_raw, current_pose = move_cartesian(
                bus=bus,
                kin=kin,
                current_raw=current_raw,
                current_pose=current_pose,
                delta=delta,
            )

            print_position(current_pose)

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，退出。")

    finally:
        close_bus(
            bus,
            disable_torque=torque_enabled,
        )


if __name__ == "__main__":
    main()
