"""
保存机械臂当前姿态（循环录入版）

用法：
    在项目根目录运行：
        python "Python代码\save_pose.py"

流程：
1. 程序打开串口，做一次安全检查
2. 你用手把机械臂掰到想要的姿态
3. 回到 PowerShell 直接按 Enter → 录入当前姿态
4. 输入这个姿态对应的按键名称（例如 1、2、3、4、stand）
5. 程序把当前姿态保存到 Python代码/poses/名称.json
6. 继续掰下一个姿态，重复 3~5
7. 输入 q 退出录入

不会移动舵机，只是读取当前位置。
"""

import json
import os

# 保证从项目根目录运行时，也能找到本文件旁边的 control_robot.py 和 calibration.json
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(SCRIPT_DIR)

import serial
import control_robot


POSE_DIR = os.path.join(SCRIPT_DIR, "poses")


# ============================================================
# RAW → 0~1 比例
# ============================================================

def raw_to_ratio(raw, item):
    """把当前 RAW 按校准数据转换成 0~1。"""

    p0 = float(item["unwrapped_at_0"])
    pm = float(item["unwrapped_at_mid"])
    p1 = float(item["unwrapped_at_1"])

    # 5号：RAW 1022 -> 551 直接线性
    if item["id"] == 5:
        if p0 == p1:
            raise ValueError("5号舵机的 0/1 校准点不能相同。")
        return (raw - p0) / (p1 - p0)

    if pm == p0 or p1 == pm:
        raise ValueError(f"ID {item['id']} 的中间校准点无效。")

    low0, high0 = sorted((p0, pm))
    low1, high1 = sorted((pm, p1))

    if low0 <= raw <= high0:
        ratio = 0.5 * (raw - p0) / (pm - p0)
    elif low1 <= raw <= high1:
        ratio = 0.5 + 0.5 * (raw - pm) / (p1 - pm)
    else:
        # 略微出界时仍按整体端点给结果，但限制到 0~1
        ratio = (raw - p0) / (p1 - p0)

    return max(0.0, min(1.0, ratio))


def read_current_pose(ser, calibration):
    ratios = []

    for name, servo_id in control_robot.JOINTS.items():
        raw = control_robot.read_position(ser, servo_id)
        item = calibration["joints"][name]
        ratio = raw_to_ratio(raw, item)

        print(
            f"  ID {servo_id} {control_robot.JOINT_LABELS[name]}："
            f"RAW {raw} → {ratio:.3f}"
        )

        ratios.append(round(ratio, 3))

    return ratios


# ============================================================
# 显示已保存的姿态
# ============================================================

def show_saved_poses():
    if not os.path.isdir(POSE_DIR):
        return

    names = sorted(
        f[:-5] for f in os.listdir(POSE_DIR) if f.lower().endswith(".json")
    )

    if not names:
        print("  （还没有任何姿态）")
        return

    print("  已保存：" + "、".join(names))


# ============================================================
# 保存一个姿态
# ============================================================

def save_pose(ratios, name):
    name = os.path.basename(name)
    if name.lower().endswith(".json"):
        name = name[:-5]

    if not name:
        print("❌ 姿态名称无效。")
        return False

    os.makedirs(POSE_DIR, exist_ok=True)
    path = os.path.join(POSE_DIR, f"{name}.json")

    # 如果已经存在同名姿态，询问是否覆盖
    if os.path.exists(path):
        overwrite = input(
            f"\n⚠️ 已存在 {name}.json，覆盖请输入 y，其他内容取消："
        ).strip().lower()
        if overwrite != "y":
            print("已取消。")
            return False

    pose = {
        "name": name,
        "ratios": ratios,
    }

    with open(path, "w", encoding="utf-8") as file:
        json.dump(pose, file, ensure_ascii=False, indent=4)

    print(f"✅ 姿态已保存：{path}")
    return True


# ============================================================
# 主程序
# ============================================================

def main():
    print("=" * 60)
    print("SCS215 姿态录入（循环模式）")
    print("=" * 60)

    calibration = control_robot.load_calibration()

    ser = serial.Serial(
        control_robot.PORT,
        control_robot.BAUDRATE,
        timeout=control_robot.TIMEOUT,
    )

    try:
        print("\n串口已打开。")

        # 启动时只做一次安全检查
        print("\n正在检查 2/5/6 号舵机当前位置是否安全……")

        if not control_robot.check_servo2_safe_position(ser, calibration):
            print("\n程序不会继续执行。")
            return
        if not control_robot.check_servo5_safe_position(ser, calibration):
            print("\n程序不会继续执行。")
            return
        if not control_robot.check_servo6_safe_position(ser, calibration):
            print("\n程序不会继续执行。")
            return

        print("\n✅ 安全检查通过。")
        print("\n当前已保存姿态：")
        show_saved_poses()

        print("\n使用说明：")
        print("  1) 用手把机械臂掰到想要的姿态")
        print("  2) 按 Enter  → 录入当前姿态")
        print("  3) 输入按键名 → 例如 1、2、3、4、stand")
        print("  4) 输入 q    → 退出录入")

        # ---------- 循环录入 ----------
        while True:
            print("\n" + "-" * 60)
            cmd = input(
                "按 Enter 录入当前姿态，输入 q 退出：\n> "
            ).strip().lower()

            if cmd == "q":
                print("\n退出录入。")
                break

            # 读取前再做一次安全检查，防止用户中途掰出范围
            if not control_robot.check_servo2_safe_position(ser, calibration):
                print("⚠️ 2号不在安全范围，本姿态未录入。")
                continue
            if not control_robot.check_servo5_safe_position(ser, calibration):
                print("⚠️ 5号不在安全范围，本姿态未录入。")
                continue
            if not control_robot.check_servo6_safe_position(ser, calibration):
                print("⚠️ 6号不在安全范围，本姿态未录入。")
                continue

            print("\n正在读取当前六个舵机位置……")
            ratios = read_current_pose(ser, calibration)

            print("\n当前姿态（0~1）：")
            print(" ".join(f"{x:.3f}" for x in ratios))

            name = input(
                "\n请输入这个姿态对应的按键 / 名称"
                "（例如 1、2、3、4、stand）：\n> "
            ).strip()

            if not name:
                print("❌ 没有输入名称，本姿态未保存。")
                continue

            save_pose(ratios, name)

        # ---------- 结束 ----------
        print("\n当前已保存姿态：")
        show_saved_poses()

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在安全退出。")

    finally:
        ser.close()
        print("串口已关闭。")
        print("程序结束。")


if __name__ == "__main__":
    main()