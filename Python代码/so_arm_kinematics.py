"""
SO-ARM 101 三关节位置运动学

比例 [0,1] ↔ 角度（rad） ↔ 笛卡尔 (x,y,z)

坐标系：
    原点在底座中心，z 向上，x 指向机械臂正前方
    只解 shoulder_pan / shoulder_lift / elbow_flex 三个关节
    wrist_flex / wrist_roll / gripper 保持不动

注意：
    如果实测位置偏差大，先现场改 [ADJUST] 块。
    方法：把机械臂摆到已知姿态，看 FK 输出和你用尺子量的是否一致。
"""

import math

# ============================================================
# [ADJUST] 连杆长度（米）——从 SO-ARM 101 URDF 提取
# ============================================================

L1 = 0.116       # 肩轴 → 肘轴
L2 = 0.135       # 肘轴 → 腕轴（这是 wrist_flex 关节的位置）
H_SHOULDER = 0.117

# ============================================================
# [ADJUST] 比例 → 角度（rad）
# 比例 0 = 最蜷缩，比例 1 = 最伸展
# 如果 IK 方向反了，把对应元组两个数互换
# ============================================================

RATIO_TO_ANGLE = {
    "shoulder_pan":  (-1.92, 1.92),
    "shoulder_lift": (-1.20, 1.20),
    "elbow_flex":    (1.69, 0.00),   # 卷曲=1.69，伸展=0
}

# URDF 关节限位（rad）——用于安全检查
JOINT_LIMITS_RAD = {
    "shoulder_pan":  (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex":    (-1.69, 1.69),
}


def ratio_to_angle(name, ratio):
    a0, a1 = RATIO_TO_ANGLE[name]
    return a0 + (a1 - a0) * max(0.0, min(1.0, ratio))


def angle_to_ratio(name, angle):
    a0, a1 = RATIO_TO_ANGLE[name]
    if a1 == a0:
        return 0.5
    r = (angle - a0) / (a1 - a0)
    return max(0.0, min(1.0, r))


def forward_kinematics(q1, q2, q3):
    """
    q1: shoulder_pan（绕 z 轴）
    q2: shoulder_lift（从水平面起算，+ 向上）
    q3: elbow_flex（相对角度，0 = 笔直，+ = 卷曲）
    返回 (x, y, z) 米
    """
    r_planar = L1 * math.cos(q2) + L2 * math.cos(q2 + q3)
    z_planar = L1 * math.sin(q2) + L2 * math.sin(q2 + q3)
    x = r_planar * math.cos(q1)
    y = r_planar * math.sin(q1)
    z = H_SHOULDER + z_planar
    return x, y, z


def ik_solve(x, y, z):
    """
    返回 [(q1,q2,q3), ...]，通常 2 个解（肘上/肘下）
    不可达时返回 []
    """
    r = math.hypot(x, y)
    z_rel = z - H_SHOULDER
    d = math.hypot(r, z_rel)

    if d > L1 + L2 - 1e-4:
        return []
    if d < abs(L1 - L2) + 1e-4:
        return []

    q1 = math.atan2(y, x)

    c3 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    c3 = max(-1.0, min(1.0, c3))
    q3_abs = math.acos(c3)

    alpha = math.atan2(z_rel, r)
    sols = []
    for q3 in (q3_abs, -q3_abs):
        beta = math.atan2(L2 * math.sin(q3),
                          L1 + L2 * math.cos(q3))
        q2 = alpha - beta
        sols.append((q1, q2, q3))
    return sols


def in_limits(q1, q2, q3):
    for q, name in ((q1, "shoulder_pan"),
                    (q2, "shoulder_lift"),
                    (q3, "elbow_flex")):
        lo, hi = JOINT_LIMITS_RAD[name]
        if not (lo - 1e-3 <= q <= hi + 1e-3):
            return False
    return True


def ratios_to_xyz(r1, r2, r3):
    q1 = ratio_to_angle("shoulder_pan", r1)
    q2 = ratio_to_angle("shoulder_lift", r2)
    q3 = ratio_to_angle("elbow_flex", r3)
    return forward_kinematics(q1, q2, q3)


def xyz_to_ratios(x, y, z, current_ratios=None, margin=0.01):
    """
    返回 [r1, r2, r3] 或 None
    current_ratios: 当前比例，用于在两个 IK 解中选离得最近的那个
    margin: 所有比例必须落在 [margin, 1-margin] 内，防止送进限位区
    """
    sols = ik_solve(x, y, z)
    if not sols:
        return None

    current_q = None
    if current_ratios is not None:
        current_q = (
            ratio_to_angle("shoulder_pan",  current_ratios[0]),
            ratio_to_angle("shoulder_lift", current_ratios[1]),
            ratio_to_angle("elbow_flex",    current_ratios[2]),
        )

    candidates = []
    for q1, q2, q3 in sols:
        if not in_limits(q1, q2, q3):
            continue

        ratios = [
            angle_to_ratio("shoulder_pan",  q1),
            angle_to_ratio("shoulder_lift", q2),
            angle_to_ratio("elbow_flex",    q3),
        ]

        # 硬边界：任何关节目标都不许靠近限位
        if any(r < margin or r > 1.0 - margin for r in ratios):
            continue

        candidates.append((ratios, (q1, q2, q3)))

    if not candidates:
        return None

    if current_q is None:
        return candidates[0][0]

    # 选关节角度上离当前位置最近的解
    def dist(item):
        q = item[1]
        return sum((a - b) ** 2 for a, b in zip(q, current_q))

    candidates.sort(key=dist)
    return candidates[0][0]

RATIO_TO_ANGLE["wrist_flex"] = (-1.65806, 1.65806)

def solve_wrist_flex(q2, q3, target_angle):
    """
    返回 wrist_flex 的 ratio，使 q2+q3+q4 = target_angle。
    撞限位时饱和到边界值，不返回 None。
    """
    q4 = target_angle - q2 - q3
    r = angle_to_ratio("wrist_flex", q4)
    return max(0.02, min(0.98, r))