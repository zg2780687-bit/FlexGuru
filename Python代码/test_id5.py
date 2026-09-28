import serial
import time

PORT = "COM7"
BAUDRATE = 1_000_000
TIMEOUT = 0.2

SERVO_ID = 5

TORQUE_ADDRESS = 0x28
GOAL_POSITION_ADDRESS = 0x2A
RUNNING_TIME_ADDRESS = 0x2C


def build_packet(servo_id, instruction, params):
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


def write_register(ser, address, value, byte_count=1):

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
        raise ValueError("只支持 1 或 2 字节")

    packet = build_packet(
        SERVO_ID,
        0x03,
        params,
    )

    print("发送：", packet.hex(" "))

    ser.write(packet)

    time.sleep(0.05)

    ser.reset_input_buffer()


def torque(ser, enabled):

    print(
        "ID5 力矩：",
        "开启" if enabled else "关闭"
    )

    write_register(
        ser,
        TORQUE_ADDRESS,
        1 if enabled else 0,
        1,
    )


def move_raw(ser, raw, time_ms=800):

    raw = int(raw) % 1024

    print(f"\nID5 → RAW {raw}")

    # 先写目标位置
    write_register(
        ser,
        GOAL_POSITION_ADDRESS,
        raw,
        2,
    )

    # 再写运行时间
    write_register(
        ser,
        RUNNING_TIME_ADDRESS,
        time_ms,
        2,
    )


def main():

    print("=" * 60)
    print("ID5 手腕旋转单舵机测试")
    print("=" * 60)

    print()
    print("校准：")
    print("0°   = RAW 205")
    print("90°  = RAW 1022")
    print("180° = RAW 785")
    print()

    ser = serial.Serial(
        PORT,
        BAUDRATE,
        timeout=TIMEOUT,
    )

    try:

        # 一开始卸力
        torque(ser, False)

        print("\n串口已打开，ID5 当前卸力。")

        # --------------------------------------------------
        # 测试 0°
        # --------------------------------------------------

        input(
            "\n请确认机械臂安全，"
            "然后按 Enter，让 ID5 到 0°（RAW 205）……"
        )

        torque(ser, True)

        move_raw(
            ser,
            205,
            800,
        )

        time.sleep(1.5)

        torque(ser, False)

        print("\n第一步完成：理论位置 0° / RAW 205")
        print("ID5 已卸力。")

        # --------------------------------------------------
        # 测试 90°
        # --------------------------------------------------

        input(
            "\n确认可以继续后，"
            "按 Enter，让 ID5 到 90°（RAW 1022）……"
        )

        torque(ser, True)

        move_raw(
            ser,
            1022,
            800,
        )

        time.sleep(1.5)

        torque(ser, False)

        print("\n第二步完成：理论位置 90° / RAW 1022")
        print("ID5 已卸力。")

        # --------------------------------------------------
        # 测试 180°
        # --------------------------------------------------

        input(
            "\n确认可以继续后，"
            "按 Enter，让 ID5 到 180°（RAW 785）……"
        )

        torque(ser, True)

        # --------------------------------------------------
        # 关键：
        #
        # 先走到 1022，
        # 再跨过 1023 -> 0，
        # 最后到 785。
        # --------------------------------------------------

        print("\n开始跨圈移动：")

        print("第 1 段：RAW 1022 → RAW 0")

        move_raw(
            ser,
            0,
            800,
        )

        time.sleep(1.5)

        print("第 2 段：RAW 0 → RAW 785")

        move_raw(
            ser,
            785,
            800,
        )

        time.sleep(1.5)

        torque(ser, False)

        print(
            "\n第三步完成：理论位置 180° / RAW 785"
        )

        print("ID5 已卸力。")

    except KeyboardInterrupt:

        print(
            "\n收到 Ctrl+C，立即卸力。"
        )

    finally:

        try:
            torque(ser, False)
        except Exception:
            pass

        ser.close()

        print("\n串口已关闭。")
        print("测试结束。")


if __name__ == "__main__":
    main()