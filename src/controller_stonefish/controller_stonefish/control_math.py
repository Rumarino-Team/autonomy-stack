import numpy as np

KP_DEFAULT = [3.50, 3.50, 2.20, 2.40, 2.40, 2.20]
KI_DEFAULT = [0.10, 0.10, 0.05, 0.05, 0.05, 0.01]
KD_DEFAULT = [0.70, 0.90, 0.40, 0.20, 0.20, 0.80]

TAM_X_Y_YAW = [
    [-1.0, 1.0, 1.0],
    [-1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [1.0, -1.0, 1.0],
]

TAM_Z_ROLL_PITCH = [
    [-1.0, 1.0, 1.0],
    [-1.0, -1.0, 1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, -1.0],
]

THRUSTOR_SATURATE_DEFAULT = 5.0


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to roll, pitch, yaw."""
    norm = np.linalg.norm([w, x, y, z])
    if norm == 0.0:
        return 0.0, 0.0, 0.0

    w /= norm
    x /= norm
    y /= norm
    z /= norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if np.abs(sinp) >= 1.0:
        pitch = np.copysign(np.pi / 2.0, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return float(roll), float(pitch), float(yaw)


def rotate_world_to_body(vector_xyz, quaternion_wxyz):
    """Rotate world-frame vector into body frame using quaternion conjugate."""
    vector = np.asarray(vector_xyz, dtype=float)
    quaternion = np.asarray(quaternion_wxyz, dtype=float)

    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        return vector.tolist()

    w, x, y, z = quaternion / norm
    # Rotation matrix from quaternion conjugate (w, -x, -y, -z)
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )

    return (rotation.T @ vector).tolist()


def mat4x3_vec3_mul(matrix_4x3, vector_3):
    return (np.asarray(matrix_4x3, dtype=float) @ np.asarray(vector_3, dtype=float)).tolist()


def clamp_and_normalize(thruster_values, saturate):
    thrusters = np.asarray(thruster_values, dtype=float)
    return (np.clip(thrusters, -saturate, saturate) / saturate).tolist()


def thrusters_from_wrench(wrench, quaternion_wxyz, saturate):
    wrench_vec = np.asarray(wrench, dtype=float)
    rotated = np.asarray(rotate_world_to_body(wrench_vec[:3], quaternion_wxyz), dtype=float)

    xy_error = np.hypot(wrench_vec[0], wrench_vec[1])
    yaw_scale = 1.0 if xy_error < 0.5 else 0.0
    input_x_y_yaw = np.array([rotated[0], rotated[1], wrench_vec[5] * yaw_scale])
    input_z_roll_pitch = np.array([wrench_vec[2], -wrench_vec[3], wrench_vec[4]])

    upper = np.asarray(mat4x3_vec3_mul(TAM_X_Y_YAW, input_x_y_yaw), dtype=float)
    lower = np.asarray(mat4x3_vec3_mul(TAM_Z_ROLL_PITCH, input_z_roll_pitch), dtype=float)
    return clamp_and_normalize(np.concatenate((upper, lower)), saturate)


def compute_thruster_command(
    current_pose,
    goal_pose,
    sum_err,
    prev_pose_err,
    dt,
    quaternion_wxyz,
    kp,
    ki,
    kd,
    saturate,
):
    current_pose_vec = np.asarray(current_pose, dtype=float)
    goal_pose_vec = np.asarray(goal_pose, dtype=float)
    sum_err_vec = np.asarray(sum_err, dtype=float)
    prev_pose_err_vec = np.asarray(prev_pose_err, dtype=float)
    kp_vec = np.asarray(kp, dtype=float)
    ki_vec = np.asarray(ki, dtype=float)
    kd_vec = np.asarray(kd, dtype=float)

    pose_err = goal_pose_vec - current_pose_vec
    pose_err[5] = normalize_angle(pose_err[5])

    if dt > 1.0e-6:
        vel_err = (pose_err - prev_pose_err_vec) / dt
        sum_err_next = sum_err_vec + pose_err * dt
    else:
        vel_err = np.zeros(6, dtype=float)
        sum_err_next = sum_err_vec.copy()

    wrench = kp_vec * pose_err + ki_vec * sum_err_next + kd_vec * vel_err

    thrusters = thrusters_from_wrench(wrench, quaternion_wxyz, saturate)
    return thrusters, pose_err.tolist(), sum_err_next.tolist()
