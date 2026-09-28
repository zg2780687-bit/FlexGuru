"""Move all six SCS215 servos to targets expressed in calibrated 0~1 units."""

import json
import os
import time

from calibration_math import ratio_to_raw, raw_to_ratio
from config import CALIBRATION_FILE, JOINTS, JOINT_LABELS, JOINT_SPEEDS
from robot_bus import close_bus, create_bus, read_all_raw, set_torque


STARTUP_POSITION_TOLERANCE = 20


def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        raise FileNotFoundError("找不到 calibration.json，请先运行 calibrate.py")
    with open(CALIBRATION_FILE, "r", encoding="utf-8") as file:
        data = json.load(file)
    missing = [name for name in JOINTS if name not in data.get("joints", {})]
    if missing:
        raise ValueError(f"校准文件缺少关节：{missing}")
    old_format = [
        name for name in JOINTS if "raw_delta" not in data["joints"][name]
    ]
    if old_format:
        raise ValueError("校准文件是旧格式，请重新运行 calibrate.py")
    wrapped = [
        name for name in JOINTS if data["joints"][name].get("crosses_wrap")
    ]
    if wrapped:
        raise ValueError(
            "校准包含跨 1023/0 的区间，禁止控制："
            f"{wrapped}。请先运行 python .\\make_safe_calibration.py"
        )
    return data


def position_is_safe_to_enable(raw, item):
    lower, upper = sorted((item["raw_at_0"], item["raw_at_1"]))
    return lower - STARTUP_POSITION_TOLERANCE <= raw <= upper + STARTUP_POSITION_TOLERANCE


def read_target():
    text = input("请输入六个 0~1 数值（空格分隔），或输入 q 退出：\n> ").strip()
    if text.lower() == "q":
        return None
    parts = text.split()
    if len(parts) != 6:
        raise ValueError("必须正好输入六个数值")
    values = [float(part) for part in parts]
    if any(value < 0.0 or value > 1.0 for value in values):
        raise ValueError("每个数值必须位于 0~1")
    return values


def main():
    calibration = load_calibration()
    bus = create_bus()
    torque_attempted = False
    try:
        bus.connect(handshake=False)
        release_failures = set_torque(bus, False)
        if release_failures:
            print("无法确认六个舵机全部卸力，拒绝继续：")
            for joint, error in release_failures:
                print(f"- {joint}: {error}")
            return
        current_raw = read_all_raw(bus)
        unsafe = [
            name
            for name in JOINTS
            if not position_is_safe_to_enable(
                current_raw[name], calibration["joints"][name]
            )
        ]
        if unsafe:
            print("\n拒绝上力：以下关节不在安全控制区间内：")
            for name in unsafe:
                item = calibration["joints"][name]
                lower, upper = sorted((item["raw_at_0"], item["raw_at_1"]))
                print(
                    f"- ID {JOINTS[name]} {JOINT_LABELS[name]}："
                    f"当前 RAW={current_raw[name]}，应手动放入 {lower}~{upper}"
                )
            print("请退出后保持舵机卸力，用 read_positions.py 辅助手动摆放。")
            return

        torque_attempted = True
        failures = set_torque(bus, True)
        if failures:
            print("部分舵机上力指令返回报警：")
            for joint, error in failures:
                print(f"- {joint}: {error}")

        print("机械臂已连接。急停时请立即切断舵机电源。")
        while True:
            current_raw = read_all_raw(bus)
            current_ratios = {
                name: raw_to_ratio(current_raw[name], calibration["joints"][name])
                for name in JOINTS
            }
            print("\n当前六关节 0~1 位置：")
            print(" ".join(f"{current_ratios[name]:.3f}" for name in JOINTS))
            print("顺序：底座 肩部 肘部 手腕俯仰 手腕旋转 夹爪")

            try:
                ratios = read_target()
            except ValueError as error:
                print(f"输入错误：{error}")
                continue
            if ratios is None:
                break

            targets = {
                name: ratio_to_raw(ratio, calibration["joints"][name])
                for name, ratio in zip(JOINTS, ratios)
            }
            largest_ratio_change = max(
                abs(ratio - current_ratios[name])
                for name, ratio in zip(JOINTS, ratios)
            )
            print("\n目标原始位置：")
            for name in JOINTS:
                print(f"ID {JOINTS[name]} {JOINT_LABELS[name]:<8} → {targets[name]}")
            if largest_ratio_change > 0.25:
                print("警告：这是较大范围运动，请确认整条路径不会碰撞。")
            if input("确认执行请输入 y：").strip().lower() != "y":
                print("已取消。")
                continue

            # Velocity-based position control avoids both repeated micro-goals
            # and model-dependent running-time units.
            bus.sync_write(
                "Running_Time",
                {name: 0 for name in JOINTS},
                normalize=False,
            )
            bus.sync_write("Goal_Velocity", JOINT_SPEEDS, normalize=False)
            bus.sync_write("Goal_Position", targets, normalize=False)

            estimated = max(
                abs(targets[name] - current_raw[name]) / JOINT_SPEEDS[name]
                for name in JOINTS
            )
            time.sleep(min(10.0, max(1.0, estimated + 0.8)))
            print("本次运动完成。")

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在安全退出。")
    except (ConnectionError, RuntimeError) as error:
        print(f"\n通信或舵机报警：{error}")
        print("请切断舵机电源，检查供电、负载和接线后再运行。")
    finally:
        close_bus(bus, disable_torque=torque_attempted)
        print("程序结束。")


if __name__ == "__main__":
    main()

