import time
import serial

PORT = "COM7"
SERVO_ID = 1

# 当前已经读到的位置
CURRENT_POSITION = 403

# 90° = 4096 / 4 = 1024
TARGET_POSITION = CURRENT_POSITION + 1024


with serial.Serial(PORT, 1_000_000, timeout=0.2) as ser:

    # 1. 打开扭矩
    ser.write(bytes.fromhex(
        "FF FF 01 04 03 28 01 CE"
    ))
    ser.read(6)

    time.sleep(0.2)

    # 2. 转到 1427
    # 1427 = 0x0593
    #
    # 你的舵机使用大端格式：
    # 05 93
    #
    # 注意：这里沿用你 write_servo.py
    # 的控制格式
    ser.write(bytes.fromhex(
        "FF FF 01 05 03 2A 05 93"
    ))

    ser.read(6)

    print(f"当前位置信息：{CURRENT_POSITION}")
    print(f"目标位置：{TARGET_POSITION}")
    print("目标转动角度：约 90°")

    time.sleep(3)

    # 3. 关闭扭矩
    ser.write(bytes.fromhex(
        "FF FF 01 04 03 28 00 CF"
    ))
    ser.read(6)

print("运行完成")