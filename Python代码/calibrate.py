"""Calibrate input 0 and input 1 separately for every joint."""

import json
import os
from datetime import datetime

from calibration_math import choose_calibration_delta, choose_safe_control_segment
from config import CALIBRATION_FILE, JOINTS, JOINT_LABELS, PORT
from robot_bus import close_bus, create_bus, read_stable_raw, set_torque


def confirm_endpoint(bus, name, description):
    print(f"\n{name}（{JOINT_LABELS[name]}）：{description}")
    input("摆放完成后按 Enter 读取位置……")
    value = read_stable_raw(bus, name)
    print(f"记录原始位置：{value}")
    return value


def main():
    print("=" * 58)
    print("SCS215 六关节逐关节端点校准")
    print("=" * 58)
    print(f"串口：{PORT}")
    print("输入 0 和输入 1 的物理方向由你决定；程序保留方向，不排序。")
    print("不要顶住机械限位，应保留安全余量。")

    if os.path.exists(CALIBRATION_FILE):
        answer = input(f"{CALIBRATION_FILE} 已存在。输入 c 覆盖并重新校准：").strip().lower()
        if answer != "c":
            print("已取消。")
            return

    bus = create_bus()
    calibration = {
        "port": PORT,
        "model": "SCS215",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "joints": {},
    }

    try:
        bus.connect(handshake=False)
        failures = set_torque(bus, False)
        if failures:
            print("部分卸力指令返回了报警；请确认机械臂能够安全手动移动：")
            for joint, error in failures:
                print(f"- {joint}: {error}")
        input("请托住机械臂，确认六个关节均已卸力，然后按 Enter 开始……")

        for name, servo_id in JOINTS.items():
            print("\n" + "=" * 58)
            print(f"校准 ID {servo_id}：{JOINT_LABELS[name]} ({name})")
            print("=" * 58)
            raw_at_0 = confirm_endpoint(
                bus,
                name,
                "移动到你希望输入 0.0 对应的安全端点",
            )
            raw_at_mid = confirm_endpoint(
                bus,
                name,
                "沿实际安全运动路径移动到中间位置（不要走另一侧）",
            )
            raw_at_1 = confirm_endpoint(
                bus,
                name,
                "移动到你希望输入 1.0 对应的安全端点",
            )

            raw_delta = choose_calibration_delta(raw_at_0, raw_at_mid, raw_at_1)

            if abs(raw_delta) < 20:
                raise ValueError(
                    f"{name} 两个端点距离过小：{raw_at_0} 与 {raw_at_1}"
                )

            safe_raw_at_0, safe_raw_at_1, was_trimmed = choose_safe_control_segment(
                raw_at_0, raw_delta
            )
            safe_delta = safe_raw_at_1 - safe_raw_at_0

            calibration["joints"][name] = {
                "id": servo_id,
                "label": JOINT_LABELS[name],
                "raw_at_0": safe_raw_at_0,
                "raw_at_mid": round((safe_raw_at_0 + safe_raw_at_1) / 2),
                "raw_at_1": safe_raw_at_1,
                "raw_delta": safe_delta,
                "crosses_wrap": False,
                "trimmed_at_wrap": was_trimmed,
                "measured_raw_at_0": raw_at_0,
                "measured_raw_at_mid": raw_at_mid,
                "measured_raw_at_1": raw_at_1,
                "measured_raw_delta": raw_delta,
            }
            print(
                f"完成：输入0 → {raw_at_0}，中点 → {raw_at_mid}，"
                f"输入1 → {raw_at_1}，有向跨度 {raw_delta:+d}"
            )
            if was_trimmed:
                print(
                    "检测到原始位置跨越 1023/0；为防止舵机绕远路，"
                    f"控制区间自动调整为 {safe_raw_at_0} → {safe_raw_at_1}。"
                )

        with open(CALIBRATION_FILE, "w", encoding="utf-8") as file:
            json.dump(calibration, file, ensure_ascii=False, indent=2)
        print(f"\n校准完成，数据已保存到：{os.path.abspath(CALIBRATION_FILE)}")
    except KeyboardInterrupt:
        print("\n校准已取消；未完成的数据不会覆盖原文件。")
    finally:
        close_bus(bus, disable_torque=True)


if __name__ == "__main__":
    main()

