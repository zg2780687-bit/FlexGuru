"""Hardware and joint configuration for the six-axis SCS215 arm."""

PORT = "COM5"
BAUDRATE = 1_000_000
CALIBRATION_FILE = "calibration.json"

# Input order used by calibration, display, and control.
JOINTS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

JOINT_LABELS = {
    "shoulder_pan": "底座旋转",
    "shoulder_lift": "肩关节",
    "elbow_flex": "肘关节",
    "wrist_flex": "手腕俯仰",
    "wrist_roll": "手腕旋转",
    "gripper": "夹爪",
}

# Encoder steps per second. The shoulder is deliberately slower because it
# carries most of the arm's weight.
JOINT_SPEEDS = {
    "shoulder_pan": 120,
    "shoulder_lift": 45,
    "elbow_flex": 80,
    "wrist_flex": 100,
    "wrist_roll": 100,
    "gripper": 80,
}

READ_RETRIES = 8
READ_RETRY_DELAY_S = 0.15

