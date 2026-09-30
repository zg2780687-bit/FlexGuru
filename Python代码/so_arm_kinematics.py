"""
SO-ARM101 尖端运动算法模块

负责：
- RAW <-> 0~1 标定值
- 0~1 <-> 关节角
- RAW <-> 关节角
- FK：关节角 -> 尖端位姿
- IK：尖端目标位姿 -> 5个关节角
- 直线轨迹点生成
- RAW 安全检查

不负责：
- 键盘输入
- 舵机通信
"""

import json
from pathlib import Path

import numpy as np
from lerobot.model.kinematics import RobotKinematics


# ============================================================
# 机械臂关节
# ============================================================

ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


# ============================================================
# SO101 关节角范围
# ============================================================

JOINT_ANGLE_LIMITS = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-96.8, 96.8),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-157.2, 162.8),
}


# ============================================================
# 工具函数
# ============================================================

def _linear(value, x0, x1, y0, y1):
    """线性插值。"""

    if x1 == x0:
        raise ValueError("标定数据中存在重复 RAW 点。")

    return y0 + (value - x0) * (y1 - y0) / (x1 - x0)


# ============================================================
# SO-ARM101 算法类
# ============================================================

class SOArmKinematics:

    def __init__(self, calibration_path, urdf_path=None):

        # 当前 Python 文件所在目录
        self.base_dir = Path(__file__).resolve().parent

        # ----------------------------------------------------
        # calibration.json
        # ----------------------------------------------------

        self.calibration_path = Path(calibration_path)

        if not self.calibration_path.is_absolute():
            self.calibration_path = (
                self.base_dir / self.calibration_path
            )

        with open(
            self.calibration_path,
            "r",
            encoding="utf-8",
        ) as file:

            self.calibration = json.load(file)

        # ----------------------------------------------------
        # URDF
        # ----------------------------------------------------

        if urdf_path is None:

            # 默认就在本文件所在目录
            urdf_path = (
                self.base_dir
                / "so101_new_calib.urdf"
            )

        self.urdf_path = Path(urdf_path)

        if not self.urdf_path.is_absolute():
            self.urdf_path = (
                self.base_dir
                / self.urdf_path
            )

        if not self.urdf_path.exists():

            raise FileNotFoundError(
                f"找不到 SO101 URDF：\n"
                f"{self.urdf_path}"
            )

        # ----------------------------------------------------
        # LeRobot Kinematics
        # ----------------------------------------------------

        self.kinematics = RobotKinematics(

            urdf_path=str(
                self.urdf_path
            ),

            target_frame_name=(
                "gripper_frame_link"
            ),

            joint_names=ARM_JOINTS,
        )

    # ========================================================
    # RAW -> 0~1
    # ========================================================

    def raw_to_ratio(
        self,
        raw,
        joint,
    ):

        item = self.calibration[
            "joints"
        ][joint]

        raw0 = item["raw_at_0"]
        raw_mid = item["raw_at_mid"]
        raw1 = item["raw_at_1"]

        raw = float(raw)

        # RAW 方向正向
        if raw0 <= raw1:

            if raw <= raw_mid:

                ratio = _linear(
                    raw,
                    raw0,
                    raw_mid,
                    0.0,
                    0.5,
                )

            else:

                ratio = _linear(
                    raw,
                    raw_mid,
                    raw1,
                    0.5,
                    1.0,
                )

        # RAW 方向反向
        else:

            if raw >= raw_mid:

                ratio = _linear(
                    raw,
                    raw0,
                    raw_mid,
                    0.0,
                    0.5,
                )

            else:

                ratio = _linear(
                    raw,
                    raw_mid,
                    raw1,
                    0.5,
                    1.0,
                )

        return float(
            np.clip(
                ratio,
                0.0,
                1.0,
            )
        )

    # ========================================================
    # 0~1 -> RAW
    # ========================================================

    def ratio_to_raw(
        self,
        ratio,
        joint,
    ):

        item = self.calibration[
            "joints"
        ][joint]

        raw0 = item["raw_at_0"]
        raw_mid = item["raw_at_mid"]
        raw1 = item["raw_at_1"]

        ratio = float(
            np.clip(
                ratio,
                0.0,
                1.0,
            )
        )

        if ratio <= 0.5:

            raw = _linear(
                ratio,
                0.0,
                0.5,
                raw0,
                raw_mid,
            )

        else:

            raw = _linear(
                ratio,
                0.5,
                1.0,
                raw_mid,
                raw1,
            )

        return int(
            round(raw)
        )

    # ========================================================
    # RAW -> 关节角
    # ========================================================

    def raw_to_joint_angles(
        self,
        raw_positions,
    ):

        angles = {}

        for joint in ARM_JOINTS:

            ratio = self.raw_to_ratio(
                raw_positions[joint],
                joint,
            )

            lower, upper = (
                JOINT_ANGLE_LIMITS[joint]
            )

            angle = (
                lower
                + ratio * (upper - lower)
            )

            angles[joint] = angle

        return angles

    # ========================================================
    # 关节角 -> RAW
    # ========================================================

    def joint_angles_to_raw(
        self,
        joint_angles,
        current_raw,
    ):

        target_raw = dict(
            current_raw
        )

        for joint in ARM_JOINTS:

            lower, upper = (
                JOINT_ANGLE_LIMITS[joint]
            )

            angle = float(
                joint_angles[joint]
            )

            if not (
                lower
                <= angle
                <= upper
            ):

                raise ValueError(
                    f"{joint} "
                    f"目标角度 {angle:.2f}° "
                    f"超出范围 "
                    f"[{lower}, {upper}]"
                )

            ratio = (
                angle - lower
            ) / (
                upper - lower
            )

            target_raw[joint] = (
                self.ratio_to_raw(
                    ratio,
                    joint,
                )
            )

        # 6号夹爪保持当前位置
        target_raw["gripper"] = (
            current_raw["gripper"]
        )

        return target_raw

    # ========================================================
    # FK
    # ========================================================

    def forward(
        self,
        joint_angles,
    ):

        q = np.array(
            [
                joint_angles[joint]
                for joint in ARM_JOINTS
            ],
            dtype=float,
        )

        return (
            self.kinematics
            .forward_kinematics(q)
        )

    # ========================================================
    # IK
    # ========================================================

    def inverse(
        self,
        current_angles,
        target_pose,
    ):
        """
        当前关节角 + 目标尖端位姿
        -> IK 解。

        当前需求只控制 XYZ，
        所以 orientation_weight = 0。
        """

        current = np.array(
            [
                current_angles[joint]
                for joint in ARM_JOINTS
            ],
            dtype=float,
        )

        result = (
            self.kinematics
            .inverse_kinematics(

                current_joint_pos=current,

                desired_ee_pose=target_pose,

                position_weight=1.0,

                orientation_weight=0.0,
            )
        )

        solution = {}

        for index, joint in enumerate(
            ARM_JOINTS
        ):

            angle = float(
                result[index]
            )

            lower, upper = (
                JOINT_ANGLE_LIMITS[joint]
            )

            if not (
                lower
                <= angle
                <= upper
            ):

                return None

            solution[joint] = angle

        # ----------------------------------------------------
        # FK 二次验证
        # ----------------------------------------------------

        actual_pose = self.forward(
            solution
        )

        actual_position = (
            actual_pose[:3, 3]
        )

        target_position = (
            target_pose[:3, 3]
        )

        error = np.linalg.norm(
            actual_position
            - target_position
        )

        # 允许 3 mm 误差
        if error > 0.003:

            return None

        return solution

    # ========================================================
    # 生成目标位姿
    # ========================================================

    @staticmethod
    def make_target_pose(
        current_pose,
        delta,
    ):

        target = (
            current_pose.copy()
        )

        target[:3, 3] += np.asarray(
            delta,
            dtype=float,
        )

        return target

    # ========================================================
    # 直线轨迹
    # ========================================================

    @staticmethod
    def interpolate_pose(
        start_pose,
        delta,
        step,
        total_steps,
    ):

        ratio = (
            step
            / total_steps
        )

        return (
            SOArmKinematics
            .make_target_pose(
                start_pose,
                np.asarray(
                    delta,
                    dtype=float,
                ) * ratio,
            )
        )

    # ========================================================
    # RAW 安全检查
    # ========================================================

    def check_raw_safe(
        self,
        raw_positions,
    ):

        for joint in ARM_JOINTS:

            item = self.calibration[
                "joints"
            ][joint]

            lower = min(
                item["raw_at_0"],
                item["raw_at_1"],
            )

            upper = max(
                item["raw_at_0"],
                item["raw_at_1"],
            )

            raw = raw_positions[
                joint
            ]

            if not (
                lower
                <= raw
                <= upper
            ):

                return False

        return True