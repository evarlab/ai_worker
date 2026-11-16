"""ContactGlove ergonomics (degrees) to conservative Sharpa flexion targets."""
import math

# SDK V2 ErgonomicsIndex order; tilt is deliberately not treated as abduction.
FLEXION = (
    ('thumb_CMC_FE', 0, 0.6), ('thumb_MCP_FE', 1, 1.0), ('thumb_IP', 2, 1.0),
    ('index_MCP_FE', 4, 1.2), ('index_PIP', 5, 1.2), ('index_DIP', 6, 1.0),
    ('middle_MCP_FE', 8, 1.2), ('middle_PIP', 9, 1.2), ('middle_DIP', 10, 1.0),
    ('ring_MCP_FE', 12, 1.2), ('ring_PIP', 13, 1.2), ('ring_DIP', 14, 1.0),
    ('pinky_MCP_FE', 16, 1.2), ('pinky_PIP', 17, 1.2), ('pinky_DIP', 18, 1.0),
)

# SDK V2 calls these MCP tilt angles. CMC abduction and pinky CMC remain held.
ABDUCTION = (
    ('thumb_MCP_AA', 3, 0.35), ('index_MCP_AA', 7, 0.35),
    ('middle_MCP_AA', 11, 0.35), ('ring_MCP_AA', 15, 0.35),
    ('pinky_MCP_AA', 19, 0.35),
)


def targets(values, gains, offsets, lower=None, upper=None, clamp=True):
    if len(values) != 20 or not all(math.isfinite(x) for x in values):
        raise ValueError('Expected 20 finite ergonomics angles in degrees')
    lower = [0.0] * len(FLEXION) if lower is None else lower
    upper = [x[2] for x in FLEXION] if upper is None else upper
    return [max(lo, min(hi, math.radians(values[index]) * gain + offset)) if clamp
            else math.radians(values[index]) * gain + offset
            for (_, index, _), gain, offset, lo, hi in
            zip(FLEXION, gains, offsets, lower, upper)]


def slew(current, target, max_step):
    return [a + max(-max_step, min(max_step, b - a)) for a, b in zip(current, target)]


def abduction_targets(values, gains, offsets, clamp=True):
    if len(values) != 20 or not all(math.isfinite(x) for x in values):
        raise ValueError('Expected 20 finite ergonomics angles in degrees')
    result = [math.radians(values[i]) * gain + offset
              for (_, i, _), gain, offset in zip(ABDUCTION, gains, offsets)]
    return [max(-limit, min(limit, value)) for value, (_, _, limit) in zip(result, ABDUCTION)] \
        if clamp else result


def ergonomics_frame(msg):
    """Adapt the official ROS bridge message to the same fields as SDK V2."""
    from types import SimpleNamespace
    values = [msg.thumb.cmc, msg.thumb.mcp, msg.thumb.ip, msg.thumb.tilt]
    for finger in (msg.index, msg.middle, msg.ring, msg.little):
        values.extend((finger.mcp, finger.pip, finger.dip, finger.tilt))
    return SimpleNamespace(id=msg.id, is_left=msg.is_left,
                           timestamp_ms=msg.generated_at_ms,
                           tracking_state=msg.tracking_state, values=values)
