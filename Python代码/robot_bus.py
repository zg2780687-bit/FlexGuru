"""Small, raw-position wrapper around the course SCS215 LeRobot adapter."""

import time
from statistics import median

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from config import JOINTS, PORT, READ_RETRIES, READ_RETRY_DELAY_S


def create_bus():
    motors = {
        name: Motor(servo_id, "scs215", MotorNormMode.RANGE_M100_100)
        for name, servo_id in JOINTS.items()
    }
    return FeetechMotorsBus(
        port=PORT,
        motors=motors,
        protocol_version=1,
    )


def read_raw(bus, joint):
    """Read one raw 0~1023 position with application-level retries."""
    last_error = None
    for _ in range(READ_RETRIES):
        try:
            value = int(
                bus.read(
                    "Present_Position",
                    joint,
                    normalize=False,
                    num_retry=2,
                )
            )
            if 0 <= value <= 1023:
                return value
            last_error = RuntimeError(f"位置越界：{value}")
        except (ConnectionError, RuntimeError) as error:
            last_error = error
        time.sleep(READ_RETRY_DELAY_S)
    raise ConnectionError(f"无法读取 {joint} 的位置") from last_error


def read_all_raw(bus):
    return {name: read_raw(bus, name) for name in JOINTS}


def read_stable_raw(bus, joint, samples=7):
    """Use the median of several samples to reject occasional bad packets."""
    values = [read_raw(bus, joint) for _ in range(samples)]
    return int(median(values))


def set_torque(bus, enabled):
    """Set torque per motor so one alarm does not prevent the others."""
    value = 1 if enabled else 0
    failures = []
    for joint in JOINTS:
        try:
            bus.write(
                "Torque_Enable",
                joint,
                value,
                normalize=False,
                num_retry=5,
            )
        except (ConnectionError, RuntimeError) as error:
            failures.append((joint, str(error)))
    return failures


def close_bus(bus, disable_torque=True):
    if not bus.is_connected:
        return
    if disable_torque:
        failures = set_torque(bus, False)
        for joint, error in failures:
            print(f"警告：{joint} 卸力未确认：{error}")
    try:
        bus.disconnect(disable_torque=False)
    except Exception as error:
        print(f"警告：关闭串口时出现问题：{error}")

