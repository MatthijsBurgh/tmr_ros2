import math

import numpy as np

_DoF = 6
_A = 0
_ALPHA = 1
_BETA = 2
_D = 3
_THETA = 4
_LLIM = 5
_ULIM = 6


def is_rotation_matrix(R: np.ndarray) -> bool:  # noqa: N803
    R_t = np.transpose(R)  # noqa: N806
    should_be_identity = np.dot(R_t, R)
    I = np.identity(3, dtype=R.dtype)  # noqa: E741, N806
    n = np.linalg.norm(I - should_be_identity)
    return n < 1e-6


def T_a_alpha(a: float, alpha: float) -> np.ndarray:  # noqa: N802
    return np.array(
        [
            [1, 0, 0, a],
            [0, math.cos(alpha), -math.sin(alpha), 0],
            [0, math.sin(alpha), math.cos(alpha), 0],
            [0, 0, 0, 1],
        ]
    )


def T_beta(beta: float) -> np.ndarray:  # noqa: N802
    return np.array(
        [
            [math.cos(beta), 0, math.sin(beta), 0],
            [0, 1, 0, 0],
            [-math.sin(beta), 0, math.cos(beta), 0],
            [0, 0, 0, 1],
        ]
    )


def T_d_theta(d: float, theta: float) -> np.ndarray:  # noqa: N802
    return np.array(
        [
            [math.cos(theta), -math.sin(theta), 0, 0],
            [math.sin(theta), math.cos(theta), 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1],
        ]
    )


def euler_angles_from_rotation_matrix(R: np.ndarray) -> np.ndarray:  # noqa: N803
    assert is_rotation_matrix(R)

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    if sy >= 1e-6:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def urdf_DH_from_tm_DH(tm_DH: list[float], tm_DeltaDH: list[float]) -> np.ndarray:  # noqa: N802, N803
    """Convert TM DH table and Delta DH to URDF DH parameters.

    Args:
        tm_DH: 42 values (7 parameters × 6 joints: theta, alpha, a, d, t, lower_limit, upper_limit)
        tm_DeltaDH: 30 values (5 parameters × 6 joints: theta, alpha, a, d, beta)

    Returns:
        urdf_DH: (DoF+1) × 7 array [a, alpha, beta, d, theta, lower_limit, upper_limit]
    """
    assert len(tm_DH) == 7 * _DoF and len(tm_DeltaDH) == 5 * _DoF

    urdf_DH = np.zeros([_DoF + 1, 7])  # noqa: N806
    for i in range(_DoF):
        urdf_DH[i, _D] = 0.001 * (tm_DH[7 * i + 3] + tm_DeltaDH[5 * i + 3])
        urdf_DH[i, _THETA] = math.radians(tm_DH[7 * i + 0] + tm_DeltaDH[5 * i + 0])
        urdf_DH[i, _LLIM] = math.radians(tm_DH[7 * i + 5])
        urdf_DH[i, _ULIM] = math.radians(tm_DH[7 * i + 6])
        urdf_DH[i + 1, _A] = 0.001 * (tm_DH[7 * i + 2] + tm_DeltaDH[5 * i + 2])
        urdf_DH[i + 1, _ALPHA] = math.radians(tm_DH[7 * i + 1] + tm_DeltaDH[5 * i + 1])
        urdf_DH[i + 1, _BETA] = math.radians(tm_DeltaDH[5 * i + 4])
    return urdf_DH


def xyzrpys_from_urdf_DH(udh: np.ndarray) -> tuple[np.ndarray, np.ndarray]:  # noqa: N802
    """Convert URDF DH parameters to xyz/rpy joint origin coordinates."""
    np.set_printoptions(suppress=True)
    xyzs = np.zeros([_DoF + 1, 3])
    rpys = np.zeros([_DoF + 1, 3])
    for i in range(_DoF + 1):
        Ta = T_a_alpha(udh[i, _A], udh[i, _ALPHA])  # noqa: N806
        Tb = T_beta(udh[i, _BETA])  # noqa: N806
        Tc = T_d_theta(udh[i, _D], udh[i, _THETA])  # noqa: N806
        T = np.dot(Ta, np.dot(Tb, Tc))  # noqa: N806
        xyzs[i] = T[0:3, 3]
        rpys[i] = euler_angles_from_rotation_matrix(T[0:3, 0:3])
    return xyzs, rpys
