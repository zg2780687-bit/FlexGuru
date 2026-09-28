"""Calibration math for the 10-bit SCS215 position register.

SCS215 position values are 0~1023 and wrap around at 1023 -> 0.

The physical movement of a joint can cross this numerical boundary.
Therefore calibration keeps an "unwrapped" continuous coordinate internally.
Only when sending a target to the servo do we convert it back to 0~1023.
"""

POSITION_MODULUS = 1024
STARTUP_TOLERANCE = 20

GRIPPER_ID = 6

# 6号夹爪禁止进入这个 RAW 区间
GRIPPER_FORBIDDEN_MIN = 140
GRIPPER_FORBIDDEN_MAX = 800

def is_gripper_raw_forbidden(raw):
    """检查6号夹爪 RAW 是否进入危险禁区。"""
    return (
        GRIPPER_FORBIDDEN_MIN
        <= raw
        <= GRIPPER_FORBIDDEN_MAX
    )

def circular_distance(a, b):
    """Shortest circular distance between two raw positions."""
    difference = abs(a - b) % POSITION_MODULUS
    return min(difference, POSITION_MODULUS - difference)


def choose_calibration_delta(raw_at_0, raw_at_mid, raw_at_1):
    """Determine the signed physical arc from input 0 to input 1.

    The midpoint tells us which of the two possible circular paths
    represents the actual physical movement.
    """
    direct = raw_at_1 - raw_at_0

    candidates = [
        direct,
        direct - POSITION_MODULUS,
        direct + POSITION_MODULUS,
    ]

    def midpoint_error(delta):
        predicted = (
            raw_at_0 + delta / 2.0
        ) % POSITION_MODULUS
        return circular_distance(predicted, raw_at_mid)

    return int(min(candidates, key=midpoint_error))


def unwrap_from_start(raw, raw_at_0, raw_delta):
    """Represent a raw position on the same continuous coordinate system.

    Example:
        raw_at_0 = 1022
        raw = 128

    For a positive calibrated direction this becomes 1152.
    """
    candidates = [
        raw + k * POSITION_MODULUS
        for k in range(-2, 3)
    ]

    if raw_delta > 0:
        return min(
            candidates,
            key=lambda value: abs(value - raw_at_0)
        )

    return min(
        candidates,
        key=lambda value: abs(value - raw_at_0)
    )


def ratio_to_unwrapped(ratio, item):
    """Convert 0~1 ratio to the continuous calibrated coordinate."""
    if not 0.0 <= ratio <= 1.0:
        raise ValueError("ratio 必须位于 0~1")

    return (
        item["raw_at_0"]
        + ratio * item["raw_delta"]
    )


def unwrapped_to_raw(unwrapped):
    """Convert continuous coordinate back to SCS215 raw 0~1023."""
    return int(round(unwrapped)) % POSITION_MODULUS


def ratio_to_raw(ratio, item):
    """Convert calibrated 0~1 ratio to actual SCS215 raw position."""
    unwrapped = ratio_to_unwrapped(ratio, item)
    return unwrapped_to_raw(unwrapped)


def raw_to_ratio(raw, item):
    """Convert an actual raw position to calibrated 0~1.

    The raw value is first unwrapped relative to the calibrated start.
    """
    raw_at_0 = item["raw_at_0"]
    raw_delta = item["raw_delta"]

    if raw_delta == 0:
        raise ValueError("校准跨度不能为 0")

    # Find the representation of raw that is closest to the
    # calibrated interval.
    candidates = [
        raw + k * POSITION_MODULUS
        for k in range(-2, 3)
    ]

    interval_end = raw_at_0 + raw_delta

    lower = min(raw_at_0, interval_end)
    upper = max(raw_at_0, interval_end)

    inside = [
        value
        for value in candidates
        if lower - STARTUP_TOLERANCE
        <= value
        <= upper + STARTUP_TOLERANCE
    ]

    if inside:
        unwrapped = min(
            inside,
            key=lambda value: abs(value - raw_at_0)
        )
    else:
        unwrapped = min(
            candidates,
            key=lambda value: abs(
                value - interval_end
            )
        )

    return (
        unwrapped - raw_at_0
    ) / raw_delta


def movement_crosses_wrap(start_raw, target_raw):
    """Return whether the direct numerical movement crosses 1023/0."""
    return abs(target_raw - start_raw) > POSITION_MODULUS / 2


def get_safe_intermediate(start_raw, target_raw):
    """Return a safe intermediate target when crossing 1023/0.

    The intermediate position stays 15 counts away from the numerical
    discontinuity.
    """
    if start_raw > 1008 and target_raw < 15:
        return 1008

    if start_raw < 15 and target_raw > 1008:
        return 15

    return None