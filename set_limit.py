"""
set_angle_range.py
SCS215 舵机角度限制设置工具
功能：自动读取当前位置 → 交互式输入最大/最小值 → 写入 EEPROM
"""

import time
import serial

# ==================== 配置区 ====================
PORT     = "COM7"
BAUDRATE = 1_000_000
TIMEOUT  = 0.2

POS_MAX   = 1023      # 位置值上限（0~1023）
ANGLE_MAX = 300.0     # SCS215 机械总角度 300°
# ===============================================


# ---------- 工具层 ----------
def angle_to_pos(angle: float) -> int:
    """角度 → 位置值（四舍五入）"""
    return int(round(angle * POS_MAX / ANGLE_MAX))


def pos_to_angle(pos: int) -> float:
    """位置值 → 角度"""
    return pos * ANGLE_MAX / POS_MAX


# ---------- 协议层 ----------
def _checksum(data) -> int:
    """SCS 协议校验和: ~(sum) & 0xFF"""
    return (~sum(data)) & 0xFF


def _build_packet(servo_id: int, instruction: int, params) -> bytes:
    """
    构建协议包:  FF FF [ID] [LEN] [CMD] [PARAMS...] [CHECKSUM]
    LEN = len(params) + 2
    """
    length = len(params) + 2
    body   = [servo_id, length, instruction, *params]
    return bytes([0xFF, 0xFF, *body, _checksum(body)])


def read_position(ser: serial.Serial, servo_id: int) -> int:
    """读取当前位置 (地址 0x38, 2 字节大端)"""
    packet = _build_packet(servo_id, 0x02, [0x38, 0x02])
    ser.write(packet)
    reply = ser.read(8)

    if len(reply) < 8 or reply[0] != 0xFF or reply[1] != 0xFF:
        raise RuntimeError(
            f"舵机 {servo_id} 应答异常: {reply.hex(' ').upper()}"
        )
    return (reply[5] << 8) | reply[6]


def write_register(ser: serial.Serial, servo_id: int, address: int,
                   value: int, byte_count: int = 1) -> None:
    """写寄存器（整数），byte_count 指定数据字节数（1 或 2）"""
    if byte_count == 1:
        params = [address, value & 0xFF]
    elif byte_count == 2:
        params = [address, (value >> 8) & 0xFF, value & 0xFF]
    else:
        raise ValueError("仅支持 1 或 2 字节写入")

    ser.write(_build_packet(servo_id, 0x03, params))
    ser.read(6)          # 丢弃应答
    time.sleep(0.1)      # 给舵机一点处理时间


def set_torque(ser: serial.Serial, servo_id: int, enable: bool) -> None:
    """0x28: 打开/关闭扭矩"""
    write_register(ser, servo_id, 0x28, 1 if enable else 0, 1)


def set_eeprom_lock(ser: serial.Serial, servo_id: int, lock: bool) -> None:
    """0x30: EEPROM 锁定(1) / 解锁(0)"""
    write_register(ser, servo_id, 0x30, 1 if lock else 0, 1)


# ---------- 业务层 ----------
def set_angle_limits(ser: serial.Serial, servo_id: int,
                     min_pos: int, max_pos: int) -> None:
    """完整流程：关闭扭矩 → 解锁 EEPROM → 写限制 → 锁定 EEPROM"""
    print(f"\n>>> 正在写入舵机 ID={servo_id}")
    print(f"    最小限制: {min_pos:>4}  ({pos_to_angle(min_pos):6.1f}°)")
    print(f"    最大限制: {max_pos:>4}  ({pos_to_angle(max_pos):6.1f}°)")
    print(f"    行程长度: {max_pos - min_pos:>4} 步  "
          f"({pos_to_angle(max_pos - min_pos):.1f}°)")

    print("  [1/5] 关闭扭矩 ...")
    set_torque(ser, servo_id, False)

    print("  [2/5] 解锁 EEPROM ...")
    set_eeprom_lock(ser, servo_id, False)

    print("  [3/5] 写入最小角度限制 (0x09) ...")
    write_register(ser, servo_id, 0x09, min_pos, 2)

    print("  [4/5] 写入最大角度限制 (0x0B) ...")
    write_register(ser, servo_id, 0x0B, max_pos, 2)

    print("  [5/5] 锁定 EEPROM ...")
    set_eeprom_lock(ser, servo_id, True)

    print(f"  ✅ 舵机 {servo_id} 设置完成")


# ---------- 交互层 ----------
def input_position(prompt: str) -> int:
    """
    读取用户输入的位置值。
      - 直接输入数字   → 视为位置值 (0~1023)
      - 输入 90d / 45d → 视为角度，自动换算为位置值
    """
    while True:
        s = input(prompt).strip()

        if s.lower().endswith('d'):
            try:
                angle = float(s[:-1])
                pos = angle_to_pos(angle)
                if 0 <= pos <= POS_MAX:
                    print(f"      → {angle}° = 位置 {pos}")
                    return pos
            except ValueError:
                pass
        else:
            try:
                pos = int(s)
                if 0 <= pos <= POS_MAX:
                    return pos
            except ValueError:
                pass

        print(f"      ⚠ 输入无效，请输入 0~{POS_MAX} 的整数，"
              f"或 '90d' 表示 90°")


def configure_servo(ser: serial.Serial) -> None:
    """对单个舵机完成一次交互式配置"""
    # 1. 读取舵机 ID
    try:
        servo_id = int(input("请输入舵机 ID: ").strip())
    except ValueError:
        print("⚠ ID 必须是整数")
        return

    # 2. 读取当前位置
    current = None
    try:
        current = read_position(ser, servo_id)
        print(f"\n舵机 {servo_id} 当前位置: {current} "
              f"({pos_to_angle(current):.1f}°)\n")
    except Exception as e:
        print(f"⚠ 读取位置失败：{e}")
        print("  （仍可继续，但请注意确认 ID 是否正确）\n")

    # 3. 输入 min / max
    min_pos = input_position("请输入最小角度限制（位置值或角度如90d）: ")
    max_pos = input_position("请输入最大角度限制（位置值或角度如90d）: ")

    if min_pos >= max_pos:
        print("❌ 最小限制必须严格小于最大限制，已取消")
        return

    # 4. 用户确认
    print(f"\n--- 预览 ---")
    print(f"  舵机 ID      : {servo_id}")
    print(f"  当前位置     : {current}")
    print(f"  最小限制     : {min_pos}")
    print(f"  最大限制     : {max_pos}")
    print(f"  行程         : {max_pos - min_pos} 步 "
          f"({pos_to_angle(max_pos - min_pos):.1f}°)")
    if input("确认写入？(y/N): ").strip().lower() != 'y':
        print("已取消")
        return

    # 5. 真正写入
    set_angle_limits(ser, servo_id, min_pos, max_pos)


def main() -> None:
    with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT) as ser:
        while True:
            configure_servo(ser)
            if input("\n继续配置其他舵机？(y/N): ").strip().lower() != 'y':
                break
    print("\n🎉 全部完成")


if __name__ == "__main__":
    main()