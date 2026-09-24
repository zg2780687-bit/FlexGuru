import serial
import time
import json


# =========================
# 基本配置
# =========================

PORT = "COM7"
BAUDRATE = 1_000_000
TIMEOUT = 0.2

SERVO_IDS = [1, 2, 3, 4, 5, 6]

CALIBRATION_FILE = "calibration.json"

# 现在没有机械臂，先使用模拟模式
# 以后机械臂连接后改成 False
SIMULATION = True


# =========================
# 打开串口
# =========================

ser = None

if not SIMULATION:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT
    )


# =========================
# 计算校验和
# =========================

def checksum(data):
    return (~sum(data)) & 0xFF


# =========================
# 读取寄存器
# =========================

def read_register(servo_id, address, byte_count):

    packet = [
        servo_id,
        4,
        0x02,
        address,
        byte_count
    ]

    packet.append(checksum(packet))

    frame = bytes([0xFF, 0xFF] + packet)

    ser.reset_input_buffer()
    ser.write(frame)

    time.sleep(0.02)

    response = ser.read(6 + byte_count)

    if len(response) < 6 + byte_count:
        raise RuntimeError(
            f"舵机 {servo_id} 返回数据长度不足："
            f"{response.hex(' ')}"
        )

    value = 0

    for i in range(byte_count):
        value = (value << 8) | response[5 + i]

    return value


# =========================
# 读取当前位置
# SCS215：当前位置地址 0x38
# =========================

def read_position(servo_id):

    return read_register(
        servo_id,
        0x38,
        2
    )


# =========================
# 模拟读取位置
# =========================

def simulation_read_position(servo_id):

    print()
    print(f"[模拟模式] 当前正在校准 ID {servo_id}")

    while True:

        value = input(
            f"请输入 ID {servo_id} 当前模拟位置（0~1023）："
        ).strip()

        try:
            value = int(value)
        except ValueError:
            print("❌ 请输入整数。")
            continue

        if not 0 <= value <= 1023:
            print("❌ 位置必须在 0~1023 之间。")
            continue

        return value


# =========================
# 获取当前位置
# =========================

def get_position(servo_id):

    if SIMULATION:
        return simulation_read_position(servo_id)

    return read_position(servo_id)


# =========================
# 开始校准
# =========================

print("=" * 50)
print("       六关节机械臂 Calibration")
print("=" * 50)

print()

if SIMULATION:

    print("当前模式：SIMULATION（模拟模式）")
    print("没有连接真实舵机。")
    print("程序会让你手动输入模拟的位置。")

else:

    print("当前模式：REAL（真实舵机模式）")
    print(f"串口：{PORT}")
    print(f"波特率：{BAUDRATE}")

print()

print("校准说明：")
print("1. 每个关节需要确定一个最小安全位置")
print("2. 每个关节需要确定一个最大安全位置")
print("3. 以后输入 0~1 时，会映射到这个范围")
print("4. 不要把机械结构移动到会卡死的位置")
print()

input("准备好后按 Enter 开始校准...")


calibration = {}


try:

    for servo_id in SERVO_IDS:

        print()
        print("=" * 50)
        print(f"正在校准舵机 ID {servo_id}")
        print("=" * 50)

        # =========================
        # 最小位置
        # =========================

        print()
        print(f"请将 ID {servo_id} 移动到【最小安全位置】")

        if SIMULATION:
            print("模拟模式：输入这个位置对应的数值。")

        input("位置确认后按 Enter...")

        min_position = get_position(servo_id)

        print(
            f"ID {servo_id} 最小位置 = {min_position}"
        )


        # =========================
        # 最大位置
        # =========================

        print()
        print(f"请将 ID {servo_id} 移动到【最大安全位置】")

        if SIMULATION:
            print("模拟模式：输入这个位置对应的数值。")

        input("位置确认后按 Enter...")

        max_position = get_position(servo_id)

        print(
            f"ID {servo_id} 最大位置 = {max_position}"
        )


        # =========================
        # 检查范围
        # =========================

        if min_position >= max_position:

            print()
            print("⚠️ 警告：最小位置 >= 最大位置")
            print(
                "这通常说明校准方向或输入数据有问题。"
            )

            print(
                f"当前：min={min_position}, "
                f"max={max_position}"
            )

            print()


        # =========================
        # 保存当前舵机校准数据
        # =========================

        calibration[str(servo_id)] = {
            "min": min_position,
            "max": max_position
        }

        print()
        print(f"ID {servo_id} 校准完成！")


    # =========================
    # 保存校准结果
    # =========================

    with open(
        CALIBRATION_FILE,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            calibration,
            f,
            indent=4,
            ensure_ascii=False
        )


    # =========================
    # 显示最终结果
    # =========================

    print()
    print("=" * 50)
    print("所有六个关节校准完成！")
    print("=" * 50)

    print()

    for servo_id in SERVO_IDS:

        data = calibration[str(servo_id)]

        print(
            f"ID {servo_id}: "
            f"{data['min']} ~ {data['max']}"
        )

    print()

    print(
        f"校准数据已保存到：{CALIBRATION_FILE}"
    )


except Exception as e:

    print()
    print("❌ 校准过程中出现错误：")
    print(e)


finally:

    if ser is not None:
        ser.close()

    print()
    print("程序结束。")