"""Convert an existing three-point calibration into safe non-wrapping ranges."""

import json
import os
import shutil

from calibration_math import choose_safe_control_segment
from config import CALIBRATION_FILE, JOINTS


BACKUP_FILE = "calibration_circular_backup.json"


def main():
    if not os.path.exists(CALIBRATION_FILE):
        raise FileNotFoundError("找不到 calibration.json，请先运行 calibrate.py")

    with open(CALIBRATION_FILE, "r", encoding="utf-8") as file:
        calibration = json.load(file)

    if not any(calibration["joints"][name].get("crosses_wrap") for name in JOINTS):
        print("当前 calibration.json 已经全部是不跨零点的安全区间，无需转换。")
        return

    if not os.path.exists(BACKUP_FILE):
        shutil.copy2(CALIBRATION_FILE, BACKUP_FILE)
        print(f"原始三点校准已备份到：{os.path.abspath(BACKUP_FILE)}")

    print("\n安全控制区间：")
    print("-" * 56)
    for name in JOINTS:
        item = calibration["joints"][name]
        measured_start = item["raw_at_0"]
        measured_mid = item.get("raw_at_mid")
        measured_end = item["raw_at_1"]
        measured_delta = item["raw_delta"]
        safe_start, safe_end, was_trimmed = choose_safe_control_segment(
            measured_start, measured_delta
        )

        item.update(
            {
                "measured_raw_at_0": measured_start,
                "measured_raw_at_mid": measured_mid,
                "measured_raw_at_1": measured_end,
                "measured_raw_delta": measured_delta,
                "raw_at_0": safe_start,
                "raw_at_mid": round((safe_start + safe_end) / 2),
                "raw_at_1": safe_end,
                "raw_delta": safe_end - safe_start,
                "crosses_wrap": False,
                "trimmed_at_wrap": was_trimmed,
            }
        )
        print(
            f"ID {item['id']} {item['label']:<8}: "
            f"{safe_start:>4} → {safe_end:<4}  跨度 {safe_end - safe_start:+d}"
        )

    with open(CALIBRATION_FILE, "w", encoding="utf-8") as file:
        json.dump(calibration, file, ensure_ascii=False, indent=2)
    print(f"\n安全校准已保存到：{os.path.abspath(CALIBRATION_FILE)}")


if __name__ == "__main__":
    main()
