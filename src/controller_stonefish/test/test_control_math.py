import math

from controller_stonefish.control_math import (
    TAM_X_Y_YAW,
    clamp_and_normalize,
    mat4x3_vec3_mul,
    normalize_angle,
    thrusters_from_wrench,
)


def test_tam_mapping_shape_and_order():
    out = mat4x3_vec3_mul(TAM_X_Y_YAW, [1.0, 0.0, 0.0])
    assert len(out) == 4
    assert out == [-1.0, -1.0, 1.0, 1.0]


def test_saturation_and_normalization_bounds():
    values = [10.0, -10.0, 3.0, -2.5, 0.0]
    out = clamp_and_normalize(values, 5.0)
    assert all(-1.0 <= v <= 1.0 for v in out)
    assert out == [1.0, -1.0, 0.6, -0.5, 0.0]


def test_yaw_gating_disabled_when_xy_error_large():
    wrench = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    out = thrusters_from_wrench(wrench, (1.0, 0.0, 0.0, 0.0), 5.0)
    expected_no_yaw = [-0.2, -0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0]
    assert out == expected_no_yaw


def test_yaw_wraparound_normalization():
    angle = math.pi + 0.1
    wrapped = normalize_angle(angle)
    assert math.isclose(wrapped, -math.pi + 0.1, rel_tol=0.0, abs_tol=1.0e-9)
