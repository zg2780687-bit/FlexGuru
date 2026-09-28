"""Calibrate the physical 0~1 range of all six SCS215 joints."""

import json
import os
from datetime import datetime

from calibration_math import choose_calibration_delta
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
    print("输入 0 和输入 1 的物理方向由你决定。")
    print("程序会保留跨 1023/0 的真实运动方向。")
    print("不要顶住机械限位，应保留安全余量。")

    if os.path.exists(CALIBRATION_FILE):
        answer = input(
            f"{CALIBRATION_FILE} 已存在。"
            "输入 c 覆盖并重新校准："
        ).strip().lower()

        if answer != "c":
            print("已取消。")
            return

    bus = create_bus()

    calibration = {
        "port": PORT,
        "model": "SCS215",
        "position_modulus": 1024,
        "created_at": datetime.now().isoformat(
            timespec="seconds"
        ),
        "joints": {},
    }

    try:
        bus.connect(handshake=False)

        failures = set_torque(bus, False)

        if failures:
            print(
                "部分卸力指令返回了报警；"
                "请确认机械臂能够安全手动移动："
            )

            for joint, error in failures:
                print(f"- {joint}: {error}")

        input(
            "请托住机械臂，确认六个关节均已卸力，"
            "然后按 Enter 开始……"
        )

        for name, servo_id in JOINTS.items():

            print("\n" + "=" * 58)
            print(
                f"校准 ID {servo_id}："
                f"{JOINT_LABELS[name]} ({name})"
            )
            print("=" * 58)

            raw_at_0 = confirm_endpoint(
                bus,
                name,
                "移动到你希望输入 0.0 对应的安全端点",
            )

            raw_at_mid = confirm_endpoint(
                bus,
                name,
                "沿实际安全运动路径移动到中间位置",
            )

            raw_at_1 = confirm_endpoint(
                bus,
                name,
                "沿同一条物理路径移动到"
                "你希望输入 1.0 对应的安全端点",
            )

            raw_delta = choose_calibration_delta(
                raw_at_0,
                raw_at_mid,
                raw_at_1,
            )

            if abs(raw_delta) < 20:
                raise ValueError(
                    f"{name} 两个端点距离过小："
                    f"{raw_at_0} 与 {raw_at_1}"
                )

            unwrapped_end = raw_at_0 + raw_delta

            crosses_wrap = not (
                0 <= unwrapped_end <= 1023
            )

            calibration["joints"][name] = {
                "id": servo_id,
                "label": JOINT_LABELS[name],

                # Physical calibration
                "raw_at_0": raw_at_0,
                "raw_at_mid": raw_at_mid,
                "raw_at_1": raw_at_1,
                "raw_delta": raw_delta,

                # Whether the continuous physical path
                # crosses the numerical 1023/0 boundary.
                "crosses_wrap": crosses_wrap,

                # Continuous coordinate of input 1.
                "unwrapped_at_0": raw_at_0,
                "unwrapped_at_1": unwrapped_end,

                # Keep the original measurements for debugging.
                "measured_raw_at_0": raw_at_0,
                "measured_raw_at_mid": raw_at_mid,
                "measured_raw_at_1": raw_at_1,
                "measured_raw_delta": raw_delta,
            }

            print(
                f"完成："
                f"输入0 → {raw_at_0}，"
                f"中点 → {raw_at_mid}，"
                f"输入1 → {raw_at_1}"
            )

            print(
                f"物理有向跨度：{raw_delta:+d}"
            )

            if crosses_wrap:
                print(
                    "检测到 1023/0 数值边界，"
                    "但不会裁剪这个物理运动范围。"
                )

                print(
                    f"内部连续坐标："
                    f"{raw_at_0} → {unwrapped_end}"
                )

        with open(
            CALIBRATION_FILE,
            "w",
            encoding="utf-8",
        ) as file:

            json.dump(
                calibration,
                file,
                ensure_ascii=False,
                indent=2,
            )

        print(
            "\n校准完成，数据已保存到："
            f"{os.path.abspath(CALIBRATION_FILE)}"
        )

    except KeyboardInterrupt:
        print(
            "\n校准已取消；"
            "未完成的数据不会覆盖原文件。"
        )

    finally:
        close_bus(
            bus,
            disable_torque=True,
        )


if __name__ == "__main__":
    main()