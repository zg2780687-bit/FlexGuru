"""Check that all six expected servo IDs answer on the configured bus."""

from config import JOINTS, JOINT_LABELS, PORT
from robot_bus import close_bus, create_bus, read_raw


def main():
    bus = create_bus()
    try:
        bus.connect(handshake=False)
        print(f"串口：{PORT}")
        print("开始检查 ID 1~6……")
        ok = True
        for name, servo_id in JOINTS.items():
            try:
                position = read_raw(bus, name)
                print(f"✓ ID {servo_id} {JOINT_LABELS[name]:<8} 原始位置={position}")
            except Exception as error:
                ok = False
                print(f"✗ ID {servo_id} {JOINT_LABELS[name]:<8} 无响应：{error}")
        print("\n六个舵机通信正常。" if ok else "\n存在未响应舵机，请先检查供电、ID 和接线。")
    finally:
        close_bus(bus, disable_torque=False)


if __name__ == "__main__":
    main()

