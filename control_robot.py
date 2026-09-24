import json


# =========================
# 配置
# =========================

CALIBRATION_FILE = "calibration.json"

# 现在没有机械臂，所以先使用模拟模式
SIMULATION = True

SERVO_IDS = [1, 2, 3, 4, 5, 6]


# =========================
# 读取校准数据
# =========================

def load_calibration():
    with open(CALIBRATION_FILE, "r", encoding="utf-8") as f:
        return json.load(f)


# =========================
# 0~1 → 实际舵机位置
# =========================

def ratio_to_position(ratio, min_position, max_position):
    return round(
        min_position + ratio * (max_position - min_position)
    )


# =========================
# 输入六个 0~1 数值
# =========================

def read_ratios():

    while True:

        text = input(
            "\n请输入 6 个关节的 0~1 数值（用空格分隔）：\n> "
        )

        parts = text.split()

        if len(parts) != 6:
            print("❌ 必须输入 6 个数字！")
            continue

        try:
            ratios = [float(x) for x in parts]
        except ValueError:
            print("❌ 输入必须全部是数字！")
            continue

        if any(x < 0 or x > 1 for x in ratios):
            print("❌ 每个数值都必须在 0~1 之间！")
            continue

        return ratios


# =========================
# 计算六个目标位置
# =========================

def calculate_targets(calibration, ratios):

    targets = {}

    for servo_id, ratio in zip(SERVO_IDS, ratios):

        data = calibration[str(servo_id)]

        min_position = data["min"]
        max_position = data["max"]

        target = ratio_to_position(
            ratio,
            min_position,
            max_position
        )

        targets[servo_id] = target

    return targets


# =========================
# 模拟发送
# =========================

def simulate_move(targets, ratios, calibration):

    print("\n========== 控制结果 ==========")

    for servo_id in SERVO_IDS:

        ratio = ratios[servo_id - 1]

        min_position = calibration[str(servo_id)]["min"]
        max_position = calibration[str(servo_id)]["max"]

        target = targets[servo_id]

        print(
            f"ID {servo_id}: "
            f"{ratio:.2f} "
            f"→ 范围 {min_position}~{max_position} "
            f"→ 目标位置 {target}"
        )

    print("==============================")

    if SIMULATION:
        print("\n[SIMULATION]")
        print("当前为模拟模式，没有向真实舵机发送指令。")


# =========================
# 主程序
# =========================

def main():

    print("=" * 50)
    print("       六关节机械臂控制程序")
    print("=" * 50)

    print()
    print("输入 6 个 0~1 数值控制六个关节。")
    print("输入 q 退出程序。")
    print()

    # 读取校准数据
    try:
        calibration = load_calibration()
    except FileNotFoundError:
        print("❌ 找不到 calibration.json")
        print("请先准备校准数据。")
        return

    # 持续控制
    while True:

        text = input(
            "请输入 6 个关节的 0~1 数值：\n> "
        ).strip()

        # 退出
        if text.lower() == "q":
            print("程序结束。")
            break

        parts = text.split()

        # 检查数量
        if len(parts) != 6:
            print("❌ 必须输入 6 个数字！")
            print()
            continue

        # 转换数字
        try:
            ratios = [float(x) for x in parts]
        except ValueError:
            print("❌ 输入必须全部是数字！")
            print()
            continue

        # 检查范围
        if any(x < 0 or x > 1 for x in ratios):
            print("❌ 每个数值都必须在 0~1 之间！")
            print()
            continue

        # 计算目标位置
        targets = calculate_targets(
            calibration,
            ratios
        )

        # 模拟运动
        simulate_move(
            targets,
            ratios,
            calibration
        )

        print()

if __name__ == "__main__":
    main()