"""
SO-ARM 101 尖端（笛卡尔）控制

用键盘操纵夹爪尖端走直线。

按键：
    w / s   →  +x / -x   （前 / 后）
    a / d   →  +y / -y   （左 / 右）
    q / e   →  +z / -z   （上 / 下）
    o       →  切换肘部 up / down
    p       →  打印当前位置
    [ / ]   →  步长减半 / 加倍
    h       →  帮助
    x       →  退出

如果 IK 无解：程序自动缩小步长，保留原方向直到
找到最近的有解位置；实在不行才放弃并提示。
"""

import os
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(SCRIPT_DIR)

import serial
import control_robot
import so_arm_kinematics as kin


# ============================================================
# 可调参数
# ============================================================

STEP_MM = 20.0          # 默认步长 2cm
MIN_STEP_MM = 2.0       # 最小步长
MAX_STEP_MM = 100.0
STEP_SCALE = 0.5        # 每次 IK 失败缩小的比例
MOVE_MS = 400           # 每步运动时长


# ============================================================
# 工具
# ============================================================

def raw_to_ratio(raw, item):
    """和 save_pose.py 一致的反向映射。"""
    p0 = float(item["unwrapped_at_0"])
    pm = float(item["unwrapped_at_mid"])
    p1 = float(item["unwrapped_at_1"])

    if item["id"] == 5:
        return (raw - p0) / (p1 - p0)

    low0, high0 = sorted((p0, pm))
    low1, high1 = sorted((pm, p1))
    if low0 <= raw <= high0:
        ratio = 0.5 * (raw - p0) / (pm - p0)
    elif low1 <= raw <= high1:
        ratio = 0.5 + 0.5 * (raw - pm) / (p1 - pm)
    else:
        ratio = (raw - p0) / (p1 - p0)
    return max(0.0, min(1.0, ratio))


def read_ratios(ser, calibration):
    out = []
    for name, sid in control_robot.JOINTS.items():
        raw = control_robot.read_position(ser, sid)
        out.append(raw_to_ratio(raw, calibration["joints"][name]))
    return out


def send_ratios(ser, ratios, duration_ms):
    calibration = control_robot.load_calibration()
    targets = control_robot.calculate_targets(ratios, calibration)

    for name, sid in control_robot.JOINTS.items():
        control_robot.set_torque(ser, sid, True)
    time.sleep(0.05)

    for name, sid in control_robot.JOINTS.items():
        control_robot.move_servo(ser, sid, targets[name], duration_ms)


def show(xyz, ratios):
    print(f"  比例: {' '.join(f'{r:.3f}' for r in ratios)}")
    print(f"  尖端 (mm): x={xyz[0]*1000:+7.1f}  "
          f"y={xyz[1]*1000:+7.1f}  z={xyz[2]*1000:+7.1f}")


# ============================================================
# 主流程
# ============================================================

def main():
    print("=" * 60)
    print("SO-ARM 101 尖端控制")
    print("=" * 60)

    calibration = control_robot.load_calibration()
    ser = serial.Serial(
        control_robot.PORT, control_robot.BAUDRATE,
        timeout=control_robot.TIMEOUT,
    )

    try:
        if not control_robot.check_servo2_safe_position(ser, calibration):
            return
        if not control_robot.check_servo5_safe_position(ser, calibration):
            return
        if not control_robot.check_servo6_safe_position(ser, calibration):
            return

        print("\n读取当前比例……")
        current = read_ratios(ser, calibration)
        q2_0 = kin.ratio_to_angle("shoulder_lift", current[1])
        q3_0 = kin.ratio_to_angle("elbow_flex",    current[2])
        q4_0 = kin.ratio_to_angle("wrist_flex",    current[3])
        WRIST_TARGET = q2_0 + q3_0 + q4_0
        print(f"夹爪朝向锁定：{WRIST_TARGET:.3f} rad")
        xyz = kin.ratios_to_xyz(*current[:3])
        show(xyz, current)

        
        step_mm = STEP_MM

        print("\n按键: w/s=±x  a/d=±y  q/e=±z  "
              "o=肘部切换  p=位置  [/]=步长  h=帮助  x=退出")

        while True:
            key = input("\n> ").strip().lower()

            if key == "x":
                break
            if key == "h":
                print("w/s ±x   a/d ±y   q/e ±z   o 肘部  p 位置  [/] 步长  x 退出")
                continue
            if key == "p":
                xyz = kin.ratios_to_xyz(*current[:3])
                show(xyz, current)
                continue
            if key == "[":
                step_mm = max(MIN_STEP_MM, step_mm / 2)
                print(f"步长 = {step_mm:.1f} mm")
                continue
            if key == "]":
                step_mm = min(MAX_STEP_MM, step_mm * 2)
                print(f"步长 = {step_mm:.1f} mm")
                continue

            direction = None
            if key == "w":   direction = ( 1,  0,  0)
            elif key == "s": direction = (-1,  0,  0)
            elif key == "a": direction = ( 0,  1,  0)
            elif key == "d": direction = ( 0, -1,  0)
            elif key == "q": direction = ( 0,  0,  1)
            elif key == "e": direction = ( 0,  0, -1)

            if direction is None:
                print("未知按键。按 h 查看帮助。")
                continue

            xyz = kin.ratios_to_xyz(*current[:3])

            # 试不同步长，从大到小
            scale = 1.0
            new3 = None
            while scale * step_mm >= MIN_STEP_MM:
                dx = direction[0] * scale * step_mm / 1000.0
                dy = direction[1] * scale * step_mm / 1000.0
                dz = direction[2] * scale * step_mm / 1000.0
                target = (xyz[0] + dx, xyz[1] + dy, xyz[2] + dz)

                print(f"  Δ目标 = ({dx*1000:+.1f}, {dy*1000:+.1f}, {dz*1000:+.1f}) mm")
                new3 = kin.xyz_to_ratios(*target, current_ratios=current[:3])
                if new3 is not None:
                    break
                scale *= STEP_SCALE

            if new3 is None:
                print("❌ 该方向完全无解，原地不动。")
                continue

            moved_mm = scale * step_mm
            q2_new = kin.ratio_to_angle("shoulder_lift", new3[1])
            q3_new = kin.ratio_to_angle("elbow_flex",    new3[2])

            r4 = kin.solve_wrist_flex(q2_new, q3_new, WRIST_TARGET)
            new_ratios = [new3[0], new3[1], new3[2], r4, current[4], current[5]]
            print(f"移动 {moved_mm:.1f} mm …")

            send_ratios(ser, new_ratios, MOVE_MS)
            time.sleep(MOVE_MS / 1000 + 0.1)

            # 从实机重新读，避免漂移
            current = read_ratios(ser, calibration)
            xyz = kin.ratios_to_xyz(*current[:3])
            show(xyz, current)

    except KeyboardInterrupt:
        print("\nCtrl+C 退出。")

    finally:
        print("\n卸力……")
        try:
            for name, sid in control_robot.JOINTS.items():
                control_robot.set_torque(ser, sid, False)
        except Exception:
            pass
        ser.close()
        print("串口已关闭。")


if __name__ == "__main__":
    main()