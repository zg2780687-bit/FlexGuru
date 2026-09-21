import serial

# ==================== 配置 ====================

PORT = "COM7"
BAUDRATE = 1_000_000
TIMEOUT = 0.2

# 两个舵机
SERVO1_ID = 1
SERVO2_ID = 2

# SCS215：0~1023 对应 0~300°
POS_MAX = 1023
ANGLE_MAX = 300.0

# 运动范围
SERVO1_ANGLE = 90.0
SERVO2_ANGLE = 45.0

# ==============================================


def angle_to_position(angle):
    """角度 → SCS215位置值"""
    return round(angle * POS_MAX / ANGLE_MAX)


def build_packet(servo_id, instruction, params):
    """生成 SCS 协议数据包"""

    length = len(params) + 2

    body = [
        servo_id,
        length,
        instruction,
        *params
    ]

    checksum = (~sum(body)) & 0xFF

    return bytes([
        0xFF,
        0xFF,
        *body,
        checksum
    ])


def write_register(ser, servo_id, address, value, byte_count=1):
    """写入寄存器"""

    if byte_count == 1:

        params = [
            address,
            value & 0xFF
        ]

    elif byte_count == 2:

        params = [
            address,
            (value >> 8) & 0xFF,
            value & 0xFF
        ]

    else:

        raise ValueError("只支持1或2字节")


    packet = build_packet(
        servo_id,
        0x03,
        params
    )

    ser.write(packet)

    # 读取应答
    ser.read(6)


def move_servo(ser, servo_id, position, time_ms=800):
    """
    控制一个舵机移动到指定位置
    """

    # 目标位置
    write_register(
        ser,
        servo_id,
        0x2A,
        position,
        2
    )

    # 运行时间
    write_register(
        ser,
        servo_id,
        0x2C,
        time_ms,
        2
    )


def main():

    print("=" * 50)
    print("       双舵机 0~1 比例控制程序")
    print("=" * 50)

    print()
    print("1号舵机：0 ~ 90°")
    print("2号舵机：0 ~ 45°")
    print()

    print("输入两个 0~1 的数字")
    print("例如：0.5 0.5")
    print("输入 q 退出")
    print()

    with serial.Serial(
        PORT,
        BAUDRATE,
        timeout=TIMEOUT
    ) as ser:

        while True:

            user_input = input(
                "请输入两个比例："
            ).strip()


            # 退出
            if user_input.lower() == "q":
                print("程序结束")
                break


            # 分割输入
            values = user_input.split()


            if len(values) != 2:

                print(
                    "❌ 请输入两个数字，例如：0.5 0.5"
                )

                continue


            try:

                ratio1 = float(values[0])
                ratio2 = float(values[1])

            except ValueError:

                print(
                    "❌ 输入必须是数字，例如：0.5 0.8"
                )

                continue


            # 检查范围
            if not 0 <= ratio1 <= 1:

                print(
                    "❌ 1号舵机的输入必须在 0~1"
                )

                continue


            if not 0 <= ratio2 <= 1:

                print(
                    "❌ 2号舵机的输入必须在 0~1"
                )

                continue


            # ==================================
            # 比例 → 角度
            # ==================================

            angle1 = ratio1 * SERVO1_ANGLE
            angle2 = ratio2 * SERVO2_ANGLE


            # ==================================
            # 角度 → 位置
            # ==================================

            position1 = angle_to_position(angle1)
            position2 = angle_to_position(angle2)


            print()
            print("---------- 控制目标 ----------")

            print(
                f"1号舵机："
                f"{ratio1:.2f}"
                f" → {angle1:.1f}°"
                f" → 位置 {position1}"
            )

            print(
                f"2号舵机："
                f"{ratio2:.2f}"
                f" → {angle2:.1f}°"
                f" → 位置 {position2}"
            )


            # ==================================
            # 同时发送两个舵机的控制指令
            # ==================================

            move_servo(
                ser,
                SERVO1_ID,
                position1
            )

            move_servo(
                ser,
                SERVO2_ID,
                position2
            )


            print("✅ 两个舵机控制指令已发送")
            print()


if __name__ == "__main__":
    main()