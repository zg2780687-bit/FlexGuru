"""
机械臂键盘姿态控制

默认按键：
    1 -> 第 1 个姿态，例如 1.json
    2 -> 第 2 个姿态，例如 2.json
    3 -> 第 3 个姿态，例如 3.json
    4 -> 第 4 个姿态，例如 4.json
    s -> stand.json
    r -> 重新显示姿态列表
    q -> 退出

姿态文件格式：
{
    "name": "stand",
    "ratios": [0.5, 0.87, 1.0, 0.5, 0.0, 0.0]
}

本程序不重新实现舵机底层通信。
所有实际运动都调用已经验证可以正常运行的 control_robot.py。
"""

import json
import os

# 保证从项目根目录运行时，control_robot.py 能正确找到 calibration.json
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(SCRIPT_DIR)

import serial
import control_robot


POSE_DIR = os.path.join(SCRIPT_DIR, "poses")


def load_poses():
    """读取 poses 文件夹中的全部 JSON 姿态。"""

    poses = {}

    if not os.path.exists(POSE_DIR):
        os.makedirs(POSE_DIR)

    for filename in sorted(os.listdir(POSE_DIR)):
        if not filename.lower().endswith(".json"):
            continue

        path = os.path.join(POSE_DIR, filename)

        try:
            with open(path, "r", encoding="utf-8") as file:
                data = json.load(file)

            ratios = data["ratios"]

            if len(ratios) != 6:
                print(f"⚠️ 跳过 {filename}：必须有 6 个数值。")
                continue

            ratios = [float(x) for x in ratios]

            if any(not 0.0 <= x <= 1.0 for x in ratios):
                print(f"⚠️ 跳过 {filename}：存在不在 0~1 范围内的数值。")
                continue

            poses[filename[:-5]] = ratios

        except Exception as error:
            print(f"⚠️ 读取 {filename} 失败：{error}")

    return poses


def show_poses(poses):
    print("\n当前可用姿态：")

    if not poses:
        print("  （还没有姿态）")
        return

    for index, (name, ratios) in enumerate(poses.items(), start=1):
        print(
            f"  {index}. {name:<15} "
            + " ".join(f"{x:.2f}" for x in ratios)
        )


def choose_pose(poses, key):
    """把键盘输入转换成姿态名称。"""

    if key == "s" and "stand" in poses:
        return "stand"

    # 1 -> 第一项，2 -> 第二项……
    if key.isdigit():
        index = int(key)

        names = list(poses.keys())

        if 1 <= index <= len(names):
            return names[index - 1]

    # 也允许直接输入姿态文件名
    if key in poses:
        return key

    return None


def main():
    print("=" * 60)
    print("SCS215 键盘姿态控制")
    print("=" * 60)

    calibration = control_robot.load_calibration()
    poses = load_poses()

    show_poses(poses)

    if not poses:
        print("\n请先使用 save_pose.py 保存姿态。")
        return

    ser = serial.Serial(
        control_robot.PORT,
        control_robot.BAUDRATE,
        timeout=control_robot.TIMEOUT,
    )

    try:
        print("\n串口已打开。")

        # 启动前做一次安全检查。
        if not control_robot.check_servo2_safe_position(ser, calibration):
            return

        if not control_robot.check_servo5_safe_position(ser, calibration):
            return

        if not control_robot.check_servo6_safe_position(ser, calibration):
            return

        print("\n安全检查通过。")
        print("\n操作：")
        print("  1/2/3/4... → 选择对应姿态")
        print("  s          → stand")
        print("  r          → 刷新姿态列表")
        print("  q          → 退出")

        while True:
            key = input("\n请输入按键：").strip().lower()

            if key == "q":
                break

            if key == "r":
                poses = load_poses()
                show_poses(poses)
                continue

            pose_name = choose_pose(poses, key)

            if pose_name is None:
                print("没有找到这个姿态。")
                continue

            ratios = poses[pose_name]

            print(f"\n选择姿态：{pose_name}")
            print("六个目标值：")
            print(" ".join(f"{x:.3f}" for x in ratios))

            # 先计算并显示目标 RAW，让使用者能看到即将执行什么。
            targets = control_robot.calculate_targets(
                ratios,
                calibration,
            )

            control_robot.print_targets(
                ratios,
                calibration,
                targets,
            )

            confirm = input(
                "\n确认执行请输入 y，其他内容取消："
            ).strip().lower()

            if confirm != "y":
                print("已取消。")
                continue

            # 执行前再次检查当前位置安全。
            if not control_robot.check_servo2_safe_position(
                ser, calibration
            ):
                print("\n本次控制已取消。")
                continue

            if not control_robot.check_servo5_safe_position(
                ser, calibration
            ):
                print("\n本次控制已取消。")
                continue

            if not control_robot.check_servo6_safe_position(
                ser, calibration
            ):
                print("\n本次控制已取消。")
                continue

            control_robot.execute_targets(
                ser,
                ratios,
                targets,
                calibration,
            )

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在安全退出。")

    finally:
        print("\n正在关闭六个舵机力矩……")

        try:
            for name, servo_id in control_robot.JOINTS.items():
                control_robot.set_torque(
                    ser,
                    servo_id,
                    False,
                )
        except Exception:
            pass

        ser.close()
        print("串口已关闭。")
        print("程序结束。")


if __name__ == "__main__":
    main()