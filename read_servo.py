import serial
import time


# =========================
# 串口配置
# =========================

SERIAL_PORT = "COM7"
BAUDRATE = 1_000_000

SERVO_IDS = [1, 2, 3, 4, 5, 6]

# SCS215 当前实际位置寄存器
POSITION_ADDRESS = 0x38


# =========================
# 计算校验和
# =========================

def calculate_checksum(data):
    return (~sum(data)) & 0xFF


# =========================
# 读取一个舵机的位置
# =========================

def read_position(ser, servo_id):

    # 读取指令
    packet = [
        servo_id,
        4,
        0x02,              # READ
        POSITION_ADDRESS,  # 0x38
        0x02               # 读取 2 个字节
    ]

    checksum = calculate_checksum(packet)

    frame = bytes(
        [0xFF, 0xFF] +
        packet +
        [checksum]
    )

    # 清空之前残留的数据
    ser.reset_input_buffer()

    # 发送
    ser.write(frame)

    # 等待舵机返回
    time.sleep(0.02)

    # 正常返回应该是 8 字节
    reply = ser.read(8)

    # 检查长度
    if len(reply) != 8:
        raise RuntimeError(
            f"返回数据长度错误："
            f"收到 {len(reply)} 字节，"
            f"数据：{reply.hex(' ')}"
        )

    # 检查帧头
    if reply[0] != 0xFF or reply[1] != 0xFF:
        raise RuntimeError(
            f"帧头错误：{reply.hex(' ')}"
        )

    # 检查 ID
    if reply[2] != servo_id:
        raise RuntimeError(
            f"ID 错误："
            f"预计 {servo_id}，"
            f"收到 {reply[2]}"
        )

    # 检查错误码
    error = reply[4]

    if error != 0:
        raise RuntimeError(
            f"舵机返回错误码：0x{error:02X}"
        )

    # =========================
    # 读取当前位置
    # reply[5] = 高 8 位
    # reply[6] = 低 8 位
    # =========================


    position = (
        (reply[5] << 8)
        | reply[6]
    )

    return position


# =========================
# 主程序
# =========================

def main():

    print("=" * 40)
    print("       六舵机当前位置读取程序")
    print("=" * 40)

    try:

        ser = serial.Serial(
            SERIAL_PORT,
            BAUDRATE,
            timeout=0.5
        )

        print(f"\n✅ 已连接 {SERIAL_PORT}")
        print(f"波特率：{BAUDRATE}")

    except Exception as e:

        print(f"\n❌ 串口打开失败：{e}")
        return

    positions = {}

    try:

        print("\n正在读取六个舵机...\n")

        for servo_id in SERVO_IDS:

            try:

                position = read_position(
                    ser,
                    servo_id
                )

                positions[servo_id] = position

                print(
                    f"ID {servo_id}: "
                    f"当前位置 = {position}"
                )

            except Exception as e:

                print(
                    f"ID {servo_id}: ❌ 读取失败"
                )

                print(
                    f"   原因：{e}"
                )

        # =========================
        # 最终汇总
        # =========================

        print("\n" + "=" * 40)
        print("          六舵机当前位置")
        print("=" * 40)

        for servo_id in SERVO_IDS:

            if servo_id in positions:

                print(
                    f"ID {servo_id}: "
                    f"{positions[servo_id]}"
                )

            else:

                print(
                    f"ID {servo_id}: "
                    f"读取失败"
                )

        print("=" * 40)

    finally:

        ser.close()

        print("\n串口已关闭。")


if __name__ == "__main__":
    main()