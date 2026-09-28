import serial
import time

PORT = "COM7"
BAUDRATE = 1_000_000
TIMEOUT = 0.2

SERVO_ID = 6

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

    else:
        params = [
            address,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]

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
        "ID6 力矩：",
        "开启" if enabled else "关闭"
    )

    write_register(
        ser,
        TORQUE_ADDRESS,
        1 if enabled else 0,
        1,
    )


def move(position):

    position = max(
        0,
        min(1023, int(position))
    )

    ser = serial.Serial(
        PORT,
        BAUDRATE,
        timeout=TIMEOUT,
    )

    try:

        torque(ser, False)

        print()
        print("ID6 测试")
        print("目标 RAW =", position)
        print()
        print("即将只控制 ID6。")
        input("确认机械臂周围安全后按 Enter……")

        torque(ser, True)

        # 先写目标位置
        write_register(
            ser,
            GOAL_POSITION_ADDRESS,
            position,
            2,
        )

        # 再写运行时间
        write_register(
            ser,
            RUNNING_TIME_ADDRESS,
            800,
            2,
        )

        print()
        print("目标已经发送。")
        time.sleep(2)

    finally:

        torque(ser, False)

        ser.close()

        print("ID6 已卸力。")
        print("串口已关闭。")


if __name__ == "__main__":

    print("=" * 50)
    print("ID6 单舵机测试")
    print("=" * 50)

    # 当前 ID6 之前读到过约 961。
    # 第一次只测试一个非常小的变化。
    move(948)
