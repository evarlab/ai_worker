import math

import pytest

from ffw_teleop.contactglove_mapping import FLEXION, slew, targets


def test_sdk_order_and_degrees():
    values = [float(i) for i in range(20)]
    mapped = targets(values, [1.0] * 15, [0.0] * 15)
    assert mapped == pytest.approx([math.radians(values[i]) for _, i, _ in FLEXION])
    assert [i for _, i, _ in FLEXION] == [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 16, 17, 18]


def test_calibration_and_clamps():
    assert targets([90.] * 20, [1.] * 15, [0.] * 15) == [x[2] for x in FLEXION]
    assert targets([90.] * 20, [-1.] * 15, [0.] * 15) == [0.] * 15
    assert targets([0.] * 20, [1.] * 15, [0.2] * 15) == [0.2] * 15


@pytest.mark.parametrize('values', [[0.] * 19, [float('nan')] * 20, [float('inf')] * 20])
def test_invalid_input(values):
    with pytest.raises(ValueError):
        targets(values, [1.] * 15, [0.] * 15)


def test_slew_from_measured_pose():
    assert slew([0.8, 0.0, 0.2], [0.0, 1.0, 0.21], 0.05) == pytest.approx([0.75, 0.05, 0.21])


def test_configured_joint_envelope():
    assert targets([90.] * 20, [1.] * 15, [0.] * 15,
                   [0.1] * 15, [0.35] * 15) == [0.35] * 15
    assert targets([-90.] * 20, [1.] * 15, [0.] * 15,
                   [0.1] * 15, [0.35] * 15) == [0.1] * 15


def test_software_angle_clamp_can_be_disabled():
    assert targets([90.] * 20, [1.] * 15, [0.] * 15,
                   clamp=False) == pytest.approx([math.pi / 2] * 15)
