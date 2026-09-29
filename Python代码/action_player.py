"""
动作播放器：按顺序执行多个预设姿态

用法：
    python "Python代码\\action_player.py" wave

读取：
    Python代码/poses/*.json
    Python代码/actions/wave.json

动作文件格式：
{
    "name": "wave",
    "steps": [
        { "pose": "lift",  "duration": 1000 },
        { "pose": "left",  "duration": 800  },
        { "pose": "right", "duration": 800  },
        { "pose": "stand", "duration": 1000 }
    ]
}
"""

import json
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(SCRIPT_DIR)

import serial
import control_robot


POSE_DIR = os.path.join(SCRIPT_DIR, "poses")
ACTION_DIR = os.path.join(SCRIPT_DIR, "actions")


def load_pose(name):
    path = os.path.join(POSE_DIR, f"{name}.json")
    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到姿态：{path}")

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    ratios = [float(x) for x in data["ratios"]]
    if len(ratios) != 6:
        raise ValueError(f"{name}.json 不是 6 个数值")
    return ratios


def load_action(name):
    path = os.path.join(ACTION_DIR, f"{name}.json")
    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到动作：{path}")

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def move_with_duration(ser, targets, duration_ms):
    """把六个舵机送到目标 RAW，用 duration_ms 毫秒完成。"""
    for name, servo_id in control_robot.JOINTS.items():
        control_robot.move_servo(
            ser, servo_id, targets[name], time_ms=duration_ms
        )


def main():
    if len(sys.argv) < 2:
        print("用法：python \"Python代码\\action_player.py\" wave")
        return

    action_name = sys.argv[1]
    action = load_action(action_name)
    steps = action["steps"]

    print("=" * 60)
    print(f"动作：{action_name}（共 {len(steps)} 步）")
    print("=" * 60)

    calibration = control_robot.load_calibration()

    ser = serial.Serial(
        control_robot.PORT,
        control_robot.BAUDRATE,
        timeout=control_robot.TIMEOUT,
    )

    try:
        print("\n串口已打开。")

        # 启动安全自检
        if not control_robot.check_servo2_safe_position(ser, calibration):
            return
        if not control_robot.check_servo5_safe_position(ser, calibration):
            return
        if not control_robot.check_servo6_safe_position(ser, calibration):
            return

        # 上力
        for name, servo_id in control_robot.JOINTS.items():
            control_robot.set_torque(ser, servo_id, True)
        time.sleep(0.1)

        for i, step in enumerate(steps, start=1):
            pose_name = step["pose"]
            duration = int(step.get("duration", 800))

            ratios = load_pose(pose_name)
            targets = control_robot.calculate_targets(ratios, calibration)

            print(f"\n[{i}/{len(steps)}] → {pose_name}  ({duration} ms)")
            for name, servo_id in control_robot.JOINTS.items():
                print(
                    f"    ID {servo_id} {control_robot.JOINT_LABELS[name]} "
                    f"→ RAW {targets[name]}"
                )

            move_with_duration(ser, targets, duration)
            time.sleep(duration / 1000 + 0.15)

        print("\n✅ 动作完成。")

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在退出。")

    finally:
        print("\n正在卸力……")
        try:
            for name, servo_id in control_robot.JOINTS.items():
                control_robot.set_torque(ser, servo_id, False)
        except Exception:
            pass
        ser.close()
        print("串口已关闭。")


if __name__ == "__main__":
    main()