"""Continuously display raw and calibrated 0~1 positions of all six servos."""

import json
import os
import time

from calibration_math import raw_to_ratio
from config import CALIBRATION_FILE, JOINTS, JOINT_LABELS
from robot_bus import close_bus, create_bus, read_all_raw, set_torque


def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        return None
    with open(CALIBRATION_FILE, "r", encoding="utf-8") as file:
        return json.load(file)


def main():
    calibration = load_calibration()
    bus = create_bus()
    try:
        bus.connect(handshake=False)
        failures = set_torque(bus, False)
        if failures:
            print("警告：以下舵机未确认卸力，请立即断电检查：")
            for joint, error in failures:
                print(f"- {joint}: {error}")
            return
        print("六个舵机已卸力，可以托住机械臂手动移动。")
        print("持续读取六个舵机；按 Ctrl+C 退出。")
        while True:
            positions = read_all_raw(bus)
            print("\nNAME             ID   RAW    RATIO")
            print("-" * 40)
            for name, servo_id in JOINTS.items():
                raw = positions[name]
                if calibration and name in calibration["joints"]:
                    ratio = raw_to_ratio(raw, calibration["joints"][name])
                    ratio_text = f"{ratio:7.3f}"
                else:
                    ratio_text = "   未校准"
                print(
                    f"{JOINT_LABELS[name]:<12} {servo_id:>2}  {raw:>4}  {ratio_text}"
                )
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n读取结束。")
    finally:
        close_bus(bus, disable_torque=False)


if __name__ == "__main__":
    main()

