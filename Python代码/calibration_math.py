"""Calibration math for the 10-bit SCS215 position register.

The sensor value can wrap while a joint is moved by hand, but the SCS215 goal
position controller must not be commanded across the 1023/0 boundary.  A
wrapped measured arc is therefore trimmed to the larger non-wrapping segment.
"""

POSITION_MODULUS = 1024
WRAP_SAFETY_MARGIN = 15


def circular_distance(a, b):
    difference = abs(a - b) % POSITION_MODULUS
    return min(difference, POSITION_MODULUS - difference)


def choose_calibration_delta(raw_at_0, raw_at_mid, raw_at_1):
    """Choose the physical arc from endpoint 0 to endpoint 1 via midpoint.

    There are two possible arcs between any two 10-bit positions. Recording a
    physical midpoint tells us whether the intended path crosses 1023 -> 0.
    """
    direct = raw_at_1 - raw_at_0
    candidates = [direct]
    if direct >= 0:
        candidates.append(direct - POSITION_MODULUS)
    else:
        candidates.append(direct + POSITION_MODULUS)

    def midpoint_error(delta):
        predicted = (raw_at_0 + delta / 2.0) % POSITION_MODULUS
        return circular_distance(predicted, raw_at_mid)

    return min(candidates, key=midpoint_error)


def choose_safe_control_segment(raw_at_0, raw_delta, margin=WRAP_SAFETY_MARGIN):
    """Return the largest part of an arc that never crosses 1023/0.

    The returned tuple is ``(safe_raw_at_0, safe_raw_at_1, was_trimmed)``.
    Direction is preserved.  ``margin`` keeps commanded positions away from
    the discontinuity itself.
    """
    unwrapped_end = raw_at_0 + raw_delta
    if 0 <= unwrapped_end <= POSITION_MODULUS - 1:
        return raw_at_0, unwrapped_end, False

    if raw_delta > 0:
        wrapped_end = unwrapped_end - POSITION_MODULUS
        candidates = [
            (raw_at_0, POSITION_MODULUS - 1 - margin),
            (margin, wrapped_end),
        ]
    else:
        wrapped_end = unwrapped_end + POSITION_MODULUS
        candidates = [
            (raw_at_0, margin),
            (POSITION_MODULUS - 1 - margin, wrapped_end),
        ]

    valid = [(start, end) for start, end in candidates if abs(end - start) >= 20]
    if not valid:
        raise ValueError("跨零点后没有足够大的安全控制区间")
    start, end = max(valid, key=lambda pair: abs(pair[1] - pair[0]))
    return int(start), int(end), True


def ratio_to_raw(ratio, item):
    raw = round(item["raw_at_0"] + ratio * item["raw_delta"])
    if not 0 <= raw < POSITION_MODULUS:
        raise ValueError(f"目标原始位置越界：{raw}")
    return raw


def raw_to_ratio(raw, item):
    """Convert a raw value using a non-wrapping calibrated interval."""
    return (raw - item["raw_at_0"]) / item["raw_delta"]

