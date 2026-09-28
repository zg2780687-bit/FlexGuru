"""
SCS215 六舵机控制程序

输入：
    6 个 0~1 的数值

含义：
    0   = 校准时记录的 0 位置
    0.5 = 校准时记录的中间位置
    1   = 校准时记录的 1 位置

修改说明：
1. 修复 2 号舵机反转问题：移除 2 号特殊分支，统一使用标准三点插值。
2. 5号舵机：以 raw=1022 为 0 点，raw=551 为 1 点，且 RAW >= 550。
3. 6号舵机：动态读取安全上限（默认 RAW <= 500）。
"""

import json
import os
import time
import serial


# ============================================================
# 基本配置
# ============================================================

PORT = "COM7"
BAUDRATE = 1_000_000
TIMEOUT = 0.2

CALIBRATION_FILE = "calibration.json"

POSITION_MODULUS = 1024


TORQUE_ADDRESS = 0x28
GOAL_POSITION_ADDRESS = 0x2A
RUNNING_TIME_ADDRESS = 0x2C

RUNNING_TIME_MS = 800


JOINTS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


JOINT_LABELS = {
    "shoulder_pan": "底座旋转",
    "shoulder_lift": "肩关节",
    "elbow_flex": "肘关节",
    "wrist_flex": "手腕俯仰",
    "wrist_roll": "手腕旋转",
    "gripper": "夹爪",
}


# ============================================================
# 数据包
# ============================================================

def build_packet(servo_id, instruction, params):
    """
    构造 SCS215 数据包。
    """
    length = len(params) + 2

    body = [
        servo_id,
        length,
        instruction,
        *params,
    ]

    checksum = (~sum(body)) & 0xFF

    return bytes([
        0xFF,
        0xFF,
        *body,
        checksum,
    ])


# ============================================================
# 写寄存器
# ============================================================

def write_register(
    ser,
    servo_id,
    address,
    value,
    byte_count=1,
):
    """
    向舵机写寄存器。
    """
    value = int(value)

    if byte_count == 1:
        params = [
            address,
            value & 0xFF,
        ]
    elif byte_count == 2:
        params = [
            address,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]
    else:
        raise ValueError("只支持 1 字节或 2 字节")

    packet = build_packet(
        servo_id,
        0x03,
        params,
    )

    ser.write(packet)
    time.sleep(0.01)

    # 清理可能残留的回复
    ser.reset_input_buffer()


# ============================================================
# 读取当前位置
# ============================================================

POSITION_ADDRESS = 0x38


def calculate_checksum(data):
    return (~sum(data)) & 0xFF


def read_position(ser, servo_id):
    """
    读取指定舵机当前位置 RAW。
    """
    packet = [
        servo_id,
        4,
        0x02,              # READ
        POSITION_ADDRESS,  # 0x38
        0x02,              # 读取2字节
    ]

    checksum = calculate_checksum(packet)

    frame = bytes([0xFF, 0xFF] + packet + [checksum])

    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.02)

    reply = ser.read(8)

    if len(reply) != 8:
        raise RuntimeError(
            f"ID {servo_id} 返回长度错误：{len(reply)} 字节，数据：{reply.hex(' ')}"
        )

    if reply[0] != 0xFF or reply[1] != 0xFF:
        raise RuntimeError(f"ID {servo_id} 帧头错误：{reply.hex(' ')}")

    if reply[2] != servo_id:
        raise RuntimeError(f"ID错误：预计 {servo_id}，收到 {reply[2]}")

    error = reply[4]

    if error != 0:
        raise RuntimeError(f"ID {servo_id} 返回错误码：0x{error:02X}")

    position = (reply[5] << 8) | reply[6]

    return position


# ============================================================
# 检查舵机安全范围
# ============================================================

def check_servo2_safe_position(ser, calibration):
    """
    检查 2 号舵机是否处于安全范围。
    """
    item = calibration["joints"]["shoulder_lift"]
    safe_min = int(item.get("safe_raw_min", 0))
    safe_max = int(item.get("safe_raw_max", 281))

    position = read_position(ser, 2)

    print(f"\n2号舵机当前 RAW：{position}")
    print(f"2号安全范围：{safe_min} ~ {safe_max}")

    if not (safe_min <= position <= safe_max):
        print("\n❌ 2号舵机当前不在安全范围内！")
        print(f"当前 RAW = {position}，允许范围 = {safe_min} ~ {safe_max}")
        print("\n❌ 为避免进入2号异常区域，本次控制不会执行。")
        return False

    print("✅ 2号舵机当前位置安全。")
    return True


def check_servo5_safe_position(ser, calibration):
    """
    检查 5 号舵机当前位置是否处于安全范围（RAW >= 550）。
    """
    item = calibration["joints"]["wrist_roll"]
    safe_min = int(item.get("safe_raw_min", 550))

    position = read_position(ser, 5)

    print(f"\n5号手腕旋转当前 RAW：{position}")
    print(f"5号安全范围：RAW >= {safe_min}")

    if position < safe_min:
        print("\n❌ 5号舵机当前不在安全范围内！")
        print(f"当前 RAW = {position}，允许最小 RAW = {safe_min}")
        print("\n❌ 为避免5号进入危险区域，程序不会继续执行。")
        return False

    print("✅ 5号舵机当前位置安全。")
    return True


def check_servo6_safe_position(ser, calibration):
    """
    检查 6 号夹爪当前位置是否处于安全范围（默认 RAW <= 500 或使用 JSON 配置）。
    """
    item = calibration["joints"]["gripper"]
    safe_max = int(item.get("safe_raw_max", 500))

    position = read_position(ser, 6)

    print(f"\n6号夹爪当前 RAW：{position}")
    print(f"6号夹爪安全范围：0 ~ {safe_max}")

    if not (0 <= position <= safe_max):
        print("\n❌ 6号夹爪当前不在安全范围内！")
        print(f"当前 RAW = {position}，允许范围 = 0 ~ {safe_max}")
        print("\n❌ 为避免夹爪进入危险区域，程序不会继续执行。")
        return False

    print("✅ 6号夹爪当前位置安全。")
    return True


# ============================================================
# 力矩控制
# ============================================================

def set_torque(ser, servo_id, enabled):
    """
    开关舵机力矩。
    """
    write_register(
        ser,
        servo_id,
        TORQUE_ADDRESS,
        1 if enabled else 0,
        1,
    )


# ============================================================
# 移动舵机
# ============================================================

def move_servo(ser, servo_id, position, time_ms=RUNNING_TIME_MS):
    """
    移动单个舵机。
    """
    position = int(position) % POSITION_MODULUS

    # 先写目标位置
    write_register(
        ser,
        servo_id,
        GOAL_POSITION_ADDRESS,
        position,
        2,
    )

    # 再写运行时间
    write_register(
        ser,
        servo_id,
        RUNNING_TIME_ADDRESS,
        time_ms,
        2,
    )


# ============================================================
# 读取校准文件
# ============================================================

def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        raise FileNotFoundError(f"找不到校准文件：{CALIBRATION_FILE}")

    with open(CALIBRATION_FILE, "r", encoding="utf-8") as file:
        data = json.load(file)

    for name in JOINTS:
        if name not in data["joints"]:
            raise ValueError(f"校准文件缺少关节：{name}")

    return data


# ============================================================
# 核心：根据三个校准点进行分段插值
# ============================================================

def ratio_to_unwrapped(ratio, item):
    """
    把 0~1 的关节比例转换成目标位置。
    """
    ratio = max(0.0, min(1.0, ratio))

    # =========================
    # 5号：0点为1022，1点为551（基于实际轨迹线性计算）
    # =========================
    if item["id"] == 5:
        start_raw = 1022.0
        end_raw = 551.0
        return start_raw + (end_raw - start_raw) * ratio

    # =========================
    # 所有其他舵机（包含2号）：通用三点分段插值
    # =========================
    start = item["unwrapped_at_0"]
    middle = item["unwrapped_at_mid"]
    end = item["unwrapped_at_1"]

    if ratio <= 0.5:
        t = ratio / 0.5
        return start + (middle - start) * t
    else:
        t = (ratio - 0.5) / 0.5
        return middle + (end - middle) * t


# ============================================================
# 连续 RAW → 实际 RAW
# ============================================================

def unwrapped_to_raw(value):
    """
    把连续坐标转换成实际的 0~1023 RAW。
    """
    return int(round(value)) % POSITION_MODULUS


def ratio_to_raw(ratio, item):
    unwrapped = ratio_to_unwrapped(ratio, item)
    return unwrapped_to_raw(unwrapped)


# ============================================================
# 计算六个目标
# ============================================================

def calculate_targets(ratios, calibration):
    targets = {}

    for ((name, servo_id), ratio) in zip(JOINTS.items(), ratios):
        item = calibration["joints"][name]
        raw = ratio_to_raw(ratio, item)
        targets[name] = raw

    return targets


# ============================================================
# 显示目标
# ============================================================

def print_targets(ratios, calibration, targets):
    print("\n目标位置：")

    for ((name, servo_id), ratio) in zip(JOINTS.items(), ratios):
        item = calibration["joints"][name]
        continuous = ratio_to_unwrapped(ratio, item)

        print(
            f"ID {servo_id} {JOINT_LABELS[name]}："
            f"{ratio:.3f} → RAW {targets[name]}"
        )
        print(f"    连续位置：{continuous:.1f}")


# ============================================================
# 输入目标
# ============================================================

def read_target():
    text = input(
        "\n请输入六个 0~1 数值（空格分隔），输入 q 退出：\n> "
    ).strip()

    if text.lower() == "q":
        return None

    parts = text.split()

    if len(parts) != 6:
        raise ValueError("必须输入正好六个数值。")

    values = []
    for part in parts:
        value = float(part)
        if not 0.0 <= value <= 1.0:
            raise ValueError("每个数值都必须在 0~1 之间。")
        values.append(value)

    return values


# ============================================================
# 执行六个舵机
# ============================================================

def execute_targets(ser, ratios, targets, calibration):
    # --------------------------------------------------------
    # 5号和6号目标位置安全检查
    # --------------------------------------------------------
    wrist_roll_raw = targets["wrist_roll"]
    safe_min_5 = int(calibration["joints"]["wrist_roll"].get("safe_raw_min", 550))
    if wrist_roll_raw < safe_min_5:
        raise RuntimeError(
            f"5号舵机目标位置不安全：RAW={wrist_roll_raw}，限制最小 RAW 为 {safe_min_5}。"
        )

    gripper_raw = targets["gripper"]
    safe_max_6 = int(calibration["joints"]["gripper"].get("safe_raw_max", 500))
    if not (0 <= gripper_raw <= safe_max_6):
        raise RuntimeError(
            f"6号夹爪目标位置不安全：RAW={gripper_raw}，允许范围为 0~{safe_max_6}。"
        )

    print("\n正在给六个舵机上力……")

    for name, servo_id in JOINTS.items():
        set_torque(ser, servo_id, True)

    time.sleep(0.05)

    print("\n正在发送目标位置……")

    for ((name, servo_id), ratio) in zip(JOINTS.items(), ratios):
        raw = targets[name]

        print(f"ID {servo_id} {JOINT_LABELS[name]} → RAW {raw}")

        move_servo(ser, servo_id, raw, RUNNING_TIME_MS)

    print("\n等待舵机运动完成……")
    time.sleep(RUNNING_TIME_MS / 1000 + 0.5)
    print("运动完成。")


# ============================================================
# 主程序
# ============================================================

def main():
    print("=" * 60)
    print("SCS215 六舵机控制程序")
    print("=" * 60)

    calibration = load_calibration()

    print(f"串口：{PORT}")
    print(f"波特率：{BAUDRATE}")

    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    print("串口已打开。")

    try:
        # 启动时卸力
        print("\n正在关闭六个舵机力矩……")
        for name, servo_id in JOINTS.items():
            set_torque(ser, servo_id, False)
        print("六个舵机已经卸力。")

        # 启动时检查安全位置
        print("\n正在检查舵机当前位置是否安全……")

        if not check_servo2_safe_position(ser, calibration):
            print("\n程序不会继续执行。")
            return

        if not check_servo5_safe_position(ser, calibration):
            print("\n程序不会继续执行。")
            return

        if not check_servo6_safe_position(ser, calibration):
            print("\n程序不会继续执行。")
            return

        print("\n✅ 2号、5号和6号当前位置均安全。")
        print("现在可以输入六个 0~1 目标值。")

        # 主循环
        while True:
            try:
                ratios = read_target()
            except ValueError as error:
                print(f"输入错误：{error}")
                continue

            if ratios is None:
                break

            targets = calculate_targets(ratios, calibration)
            print_targets(ratios, calibration, targets)

            confirm = input(
                "\n确认执行请输入 y，输入其他内容取消："
            ).strip().lower()

            if confirm != "y":
                print("已取消。")
                continue

            # 执行前再次检查安全位置
            if not check_servo2_safe_position(ser, calibration):
                print("\n本次控制已取消。")
                continue

            if not check_servo5_safe_position(ser, calibration):
                print("\n本次控制已取消。")
                continue

            if not check_servo6_safe_position(ser, calibration):
                print("\n本次控制已取消。")
                continue

            execute_targets(ser, ratios, targets, calibration)

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在安全退出。")

    finally:
        print("\n正在关闭六个舵机力矩……")
        try:
            for name, servo_id in JOINTS.items():
                set_torque(ser, servo_id, False)
        except Exception:
            pass

        ser.close()
        print("串口已关闭。")
        print("程序结束。")


if __name__ == "__main__":
    main()