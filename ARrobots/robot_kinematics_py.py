"""
robot_kinematics_py.py - pure Python twin of ARrobots/src/kinematics.cpp.

AR4.py normally loads the compiled module (robot_kinematics.pyd / .so built from
ARrobots/src). This file exposes the same functions so the program still runs when that
build is missing for the current Python version or platform, or when the compiled module
is older than the solver source (AR4.py checks API_VERSION). It is also the reference
used to test the C++ solver: both must give the same answers.

Conventions (identical to the C++ and the Teensy firmware):
  * joints are degrees, in the robot's own order J1..J6
  * a pose is [x, y, z, u, v, w]: mm and a rotation vector (axis * angle). forward_kinematics
    returns the rotation vector in radians; SolveInverseKinematics takes it in degrees.
  * the DH table is Craig's modified convention: Rx(alpha) Tx(a) Rz(theta + joint) Tz(d)
"""

import math

API_VERSION = 2
IS_FALLBACK = True

ROBOT_nDOFs = 6
_TABLE = 6

# Robot_Data layout from kinematics.cpp: 6 DH rows of [alpha, a, theta, d, 0, 0], then base
# xyzwpr (36..41), tool xyzwpr (42..47), upper limits (48..53), lower limit magnitudes
# (54..59) and joint senses (60..65).
_DHM_ALPHA, _DHM_A, _DHM_THETA, _DHM_D = 0, 1, 2, 3
_DATA = [0.0] * (11 * _TABLE)
_BASE = 6 * _TABLE
_TOOL = 7 * _TABLE
_UPPER = 8 * _TABLE
_LOWER = 9 * _TABLE
_SENSE = 10 * _TABLE

_joint_pos_limit = [0.0] * ROBOT_nDOFs
_joint_neg_limit = [0.0] * ROBOT_nDOFs

KinematicError = 0

# --- solver tuning (keep in step with kinematics.cpp / the Teensy sketch) -----------------
IK_SING_SIN_EPS = 1e-4     # |sin J5| below this: wrist singular (J5 < 0.006 deg)
IK_DUP_TOL_DEG = 1e-3      # solutions closer than this on every joint are one solution


# ------------------------------------------------------------------------------- setup
def _dh(row, col):
    return _DATA[row * _TABLE + col]


def _set_dh(row, col, value):
    _DATA[row * _TABLE + col] = float(value)


def robot_data_reset():
    for i in range(6):
        _DATA[_BASE + i] = 0.0
        _DATA[_TOOL + i] = 0.0
    for i in range(ROBOT_nDOFs):
        _DATA[_SENSE + i] = 1.0
        _DATA[_LOWER + i] = 180.0
        _DATA[_UPPER + i] = 180.0
    for alpha, row in zip((0.0, -0.5 * math.pi, 0.0, -0.5 * math.pi, 0.5 * math.pi, -0.5 * math.pi), range(6)):
        _set_dh(row, _DHM_ALPHA, alpha)
    for theta, row in zip((0.0, 0.0, -0.5 * math.pi, 0.0, 0.0, math.pi), range(6)):
        _set_dh(row, _DHM_THETA, theta)
    for row in range(6):
        _set_dh(row, _DHM_A, 0.0)
        _set_dh(row, _DHM_D, 0.0)


def robot_set():
    """AR4 default geometry (the same numbers the Teensy firmware ships with)."""
    robot_data_reset()
    for alpha, row in zip((0.0, -0.5 * math.pi, 0.0, -0.5 * math.pi, 0.5 * math.pi, -0.5 * math.pi), range(6)):
        _set_dh(row, _DHM_ALPHA, alpha)
    for theta, row in zip((0.0, -0.5 * math.pi, 0.0, 0.0, 0.0, math.pi), range(6)):
        _set_dh(row, _DHM_THETA, theta)
    _set_dh(1, _DHM_A, 64.2)
    _set_dh(2, _DHM_A, 305.0)
    _set_dh(0, _DHM_D, 169.77)
    _set_dh(1, _DHM_D, 0.0)
    _set_dh(3, _DHM_D, 222.63)
    _set_dh(5, _DHM_D, 41.0)
    for i, (up, lo) in enumerate(((170, 170), (90, 42), (52, 89), (180, 180), (105, 105), (180, 180))):
        _DATA[_UPPER + i] = float(up)
        _DATA[_LOWER + i] = float(lo)


def set_dh_parameters_explicit(theta1, theta2, theta3, theta4, theta5, theta6,
                               alpha1, alpha2, alpha3, alpha4, alpha5, alpha6,
                               a1, a2, a3, a4, a5, a6,
                               d1, d2, d3, d4, d5, d6):
    """Angles in radians, lengths in mm (same signature as the compiled module)."""
    for row, v in enumerate((theta1, theta2, theta3, theta4, theta5, theta6)):
        _set_dh(row, _DHM_THETA, v)
    for row, v in enumerate((alpha1, alpha2, alpha3, alpha4, alpha5, alpha6)):
        _set_dh(row, _DHM_ALPHA, v)
    for row, v in enumerate((a1, a2, a3, a4, a5, a6)):
        _set_dh(row, _DHM_A, v)
    for row, v in enumerate((d1, d2, d3, d4, d5, d6)):
        _set_dh(row, _DHM_D, v)


def get_dh_parameters():
    return [[_dh(r, _DHM_THETA), _dh(r, _DHM_ALPHA), _dh(r, _DHM_A), _dh(r, _DHM_D)] for r in range(6)]


def set_joint_limits(pos_limits, neg_limits):
    if len(pos_limits) != ROBOT_nDOFs or len(neg_limits) != ROBOT_nDOFs:
        raise RuntimeError(f"Expected positive and negative limit vectors of size {ROBOT_nDOFs}")
    for i in range(ROBOT_nDOFs):
        _joint_pos_limit[i] = float(pos_limits[i])
        _joint_neg_limit[i] = float(neg_limits[i])
        _DATA[_UPPER + i] = float(pos_limits[i])
        _DATA[_LOWER + i] = float(neg_limits[i])


def get_joint_limits():
    return list(_joint_pos_limit), list(_joint_neg_limit)


def set_robot_tool_frame(x, y, z, rz_deg, ry_deg, rx_deg):
    _DATA[_TOOL + 0] = float(x)
    _DATA[_TOOL + 1] = float(y)
    _DATA[_TOOL + 2] = float(z)
    _DATA[_TOOL + 3] = math.radians(float(rz_deg))
    _DATA[_TOOL + 4] = math.radians(float(ry_deg))
    _DATA[_TOOL + 5] = math.radians(float(rx_deg))


def get_robot_tool_frame():
    t = _DATA[_TOOL:_TOOL + 6]
    return [t[0], t[1], t[2], math.degrees(t[3]), math.degrees(t[4]), math.degrees(t[5])]


def kinematic_error():
    return KinematicError


# ------------------------------------------------------------------- 4x4 (column-major)
def _eye():
    m = [0.0] * 16
    m[0] = m[5] = m[10] = m[15] = 1.0
    return m


def _mul(a, b):
    """Homogeneous product a*b for flat column-major 4x4 with [0 0 0 1] bottom row."""
    o = [0.0] * 16
    for c in range(3):
        o[4 * c + 0] = a[0] * b[4 * c] + a[4] * b[4 * c + 1] + a[8] * b[4 * c + 2]
        o[4 * c + 1] = a[1] * b[4 * c] + a[5] * b[4 * c + 1] + a[9] * b[4 * c + 2]
        o[4 * c + 2] = a[2] * b[4 * c] + a[6] * b[4 * c + 1] + a[10] * b[4 * c + 2]
    o[12] = a[0] * b[12] + a[4] * b[13] + a[8] * b[14] + a[12]
    o[13] = a[1] * b[12] + a[5] * b[13] + a[9] * b[14] + a[13]
    o[14] = a[2] * b[12] + a[6] * b[13] + a[10] * b[14] + a[14]
    o[15] = 1.0
    return o


def _inv(m):
    o = [0.0] * 16
    o[0], o[1], o[2] = m[0], m[4], m[8]
    o[4], o[5], o[6] = m[1], m[5], m[9]
    o[8], o[9], o[10] = m[2], m[6], m[10]
    o[12] = -(m[0] * m[12] + m[1] * m[13] + m[2] * m[14])
    o[13] = -(m[4] * m[12] + m[5] * m[13] + m[6] * m[14])
    o[14] = -(m[8] * m[12] + m[9] * m[13] + m[10] * m[14])
    o[15] = 1.0
    return o


def _xyzwpr_2_pose(v):
    """transl(x,y,z) * rotx(v[3]) * roty(v[4]) * rotz(v[5]) (radians)."""
    srx, crx = math.sin(v[3]), math.cos(v[3])
    sry, cry = math.sin(v[4]), math.cos(v[4])
    srz, crz = math.sin(v[5]), math.cos(v[5])
    p = [0.0] * 16
    p[0] = cry * crz
    p[4] = -cry * srz
    p[8] = sry
    p[12] = v[0]
    p[1] = crx * srz + crz * srx * sry
    p[5] = crz * crx - srx * sry * srz
    p[9] = -cry * srx
    p[13] = v[1]
    p[2] = srx * srz - crz * crx * sry
    p[6] = crz * srx + crx * sry * srz
    p[10] = crx * cry
    p[14] = v[2]
    p[15] = 1.0
    return p


def _dhm_2_pose(rx, tx, rz, tz):
    crx, srx = math.cos(rx), math.sin(rx)
    crz, srz = math.cos(rz), math.sin(rz)
    p = [0.0] * 16
    p[0] = crz
    p[4] = -srz
    p[8] = 0.0
    p[12] = tx
    p[1] = crx * srz
    p[5] = crx * crz
    p[9] = -srx
    p[13] = -tz * srx
    p[2] = srx * srz
    p[6] = crz * srx
    p[10] = crx
    p[14] = tz * crx
    p[15] = 1.0
    return p


def _pose_2_xyzuvw(pose):
    """Rotation matrix -> rotation vector (radians); stable at 0 and 180 degrees."""
    r00, r10, r20 = pose[0], pose[1], pose[2]
    r01, r11, r21 = pose[4], pose[5], pose[6]
    r02, r12, r22 = pose[8], pose[9], pose[10]
    v = [r21 - r12, r02 - r20, r10 - r01]
    vnorm = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    c = max(-1.0, min(1.0, ((r00 + r11 + r22) - 1.0) * 0.5))
    angle = math.atan2(0.5 * vnorm, c)
    axis = [0.0, 0.0, 0.0]
    if angle < 1e-9:
        pass
    elif c > -0.5:
        axis = [v[0] / vnorm, v[1] / vnorm, v[2] / vnorm]
    else:
        S = [[r00 - c, 0.5 * (r01 + r10), 0.5 * (r02 + r20)],
             [0.5 * (r01 + r10), r11 - c, 0.5 * (r12 + r21)],
             [0.5 * (r02 + r20), 0.5 * (r12 + r21), r22 - c]]
        k = 0
        if S[1][1] > S[k][k]:
            k = 1
        if S[2][2] > S[k][k]:
            k = 2
        col = [S[0][k], S[1][k], S[2][k]]
        n = math.sqrt(col[0] ** 2 + col[1] ** 2 + col[2] ** 2)
        axis = [col[0] / n, col[1] / n, col[2] / n]
        if axis[0] * v[0] + axis[1] * v[1] + axis[2] * v[2] < 0.0:
            axis = [-axis[0], -axis[1], -axis[2]]
    return [pose[12], pose[13], pose[14], axis[0] * angle, axis[1] * angle, axis[2] * angle]


def _xyzuvw_2_pose(x):
    """Rotation vector (radians) + position -> pose."""
    s = math.sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5])
    p = _eye()
    if s >= 1e-6 * math.pi / 180.0 and max(abs(x[3]), abs(x[4]), abs(x[5])) >= 1e-6:
        kx, ky, kz = x[3] / s, x[4] / s, x[5] / s
        c, sn = math.cos(s), math.sin(s)
        p[0] = kx * kx + c * (1.0 - kx * kx)
        p[4] = kx * ky * (1.0 - c) - kz * sn
        p[8] = kx * kz * (1.0 - c) + ky * sn
        p[1] = kx * ky * (1.0 - c) + kz * sn
        p[5] = ky * ky + (1.0 - ky * ky) * c
        p[9] = ky * kz * (1.0 - c) - kx * sn
        p[2] = kx * kz * (1.0 - c) - ky * sn
        p[6] = ky * kz * (1.0 - c) + kx * sn
        p[10] = kz * kz + (1.0 - kz * kz) * c
    p[12], p[13], p[14] = x[0], x[1], x[2]
    return p


# ------------------------------------------------------------------------ forward kinematics
def _forward_arm(joints):
    pose = _xyzwpr_2_pose(_DATA[_BASE:_BASE + 6])
    for i in range(ROBOT_nDOFs):
        ji = joints[i] * _DATA[_SENSE + i] * math.pi / 180.0
        pose = _mul(pose, _dhm_2_pose(_dh(i, _DHM_ALPHA), _dh(i, _DHM_A), _dh(i, _DHM_THETA) + ji, _dh(i, _DHM_D)))
    return _mul(pose, _xyzwpr_2_pose(_DATA[_TOOL:_TOOL + 6]))


def forward_kinematics(joints):
    """Joints (deg) -> [x, y, z, u, v, w] with the rotation vector in radians."""
    if len(joints) != ROBOT_nDOFs:
        raise RuntimeError("Expected 6 joint angles")
    return _pose_2_xyzuvw(_forward_arm([float(j) for j in joints]))


# ------------------------------------------------------------------------ inverse kinematics
def _ik_raw(pose, DK, joints_approx_in):
    """Closed-form IK of the arm for one wrist branch (sign of the J5 estimate).
    Faithful port of inverse_kinematics_raw(); returns joints in degrees or None."""
    joints_approx = [DK[60 + i] * joints_approx_in[i] for i in range(6)]
    base = _xyzwpr_2_pose(DK[36:42])
    tool = _xyzwpr_2_pose(DK[42:48])
    Hout = [0.0] * 16
    b_Hout = [0.0] * 9
    for i0 in range(4):
        i = i0 << 2
        Hout[i] = base[i0]
        Hout[1 + i] = base[i0 + 4]
        Hout[2 + i] = base[i0 + 8]
        Hout[3 + i] = base[i0 + 12]
    for i0 in range(3):
        i = i0 << 2
        Hout[3 + i] = 0.0
        b_Hout[3 * i0] = -Hout[i]
        b_Hout[1 + 3 * i0] = -Hout[1 + i]
        b_Hout[2 + 3 * i0] = -Hout[2 + i]
    for i0 in range(3):
        Hout[12 + i0] = b_Hout[i0] * base[12] + b_Hout[i0 + 3] * base[13] + b_Hout[i0 + 6] * base[14]
    for i0 in range(4):
        i = i0 << 2
        base[i] = tool[i0]
        base[1 + i] = tool[i0 + 4]
        base[2 + i] = tool[i0 + 8]
        base[3 + i] = tool[i0 + 12]
    for i0 in range(3):
        i = i0 << 2
        base[3 + i] = 0.0
        b_Hout[3 * i0] = -base[i]
        b_Hout[1 + 3 * i0] = -base[1 + i]
        b_Hout[2 + 3 * i0] = -base[2 + i]
    for i0 in range(3):
        base[12 + i0] = b_Hout[i0] * tool[12] + b_Hout[i0 + 3] * tool[13] + b_Hout[i0 + 6] * tool[14]

    dv0 = [0.0, 0.0, -DK[33], 1.0]
    c_Hout = [0.0] * 16
    P04 = [0.0] * 4
    for i0 in range(4):
        for i in range(4):
            i1 = i << 2
            c_Hout[i0 + i1] = (Hout[i0] * pose[i1] + Hout[i0 + 4] * pose[1 + i1]
                               + Hout[i0 + 8] * pose[2 + i1] + Hout[i0 + 12] * pose[3 + i1])
        P04[i0] = 0.0
        for i in range(4):
            i1 = i << 2
            v = (c_Hout[i0] * base[i1] + c_Hout[i0 + 4] * base[1 + i1]
                 + c_Hout[i0 + 8] * base[2 + i1] + c_Hout[i0 + 12] * base[3 + i1])
            tool[i0 + i1] = v
            P04[i0] += v * dv0[i]

    if DK[9] == 0.0:
        q1 = math.atan2(P04[1], P04[0])
    else:
        ms = P04[0] * P04[0] + P04[1] * P04[1] - DK[9] * DK[9]
        if ms < 0.0:
            return None
        q1 = math.atan2(P04[1], P04[0]) - math.atan2(DK[9], math.sqrt(ms))

    k2 = P04[2] - DK[3]
    k1 = (math.cos(q1) * P04[0] + math.sin(q1) * P04[1]) - DK[7]
    ai = (((k1 * k1 + k2 * k2) - DK[13] * DK[13]) - DK[21] * DK[21]) - DK[19] * DK[19]
    B = 2.0 * DK[21] * DK[13]
    C = 2.0 * DK[19] * DK[13]
    s31 = 0.0
    c31 = 0.0
    if C == 0.0:
        s31 = -ai / B
        ms = 1.0 - s31 * s31
        if ms >= 0.0:
            c31 = math.sqrt(ms)
    else:
        q13_idx_2 = C * C
        bb_div_cc = B * B / q13_idx_2
        ms = 2.0 * ai * B / q13_idx_2
        ms = ms * ms - 4.0 * ((1.0 + bb_div_cc) * (ai * ai / q13_idx_2 - 1.0))
        if ms >= 0.0:
            s31 = (-2.0 * ai * B / q13_idx_2 + math.sqrt(ms)) / (2.0 * (1.0 + bb_div_cc))
            c31 = (ai + B * s31) / C

    if not (ms >= 0.0 and abs(s31) <= 1.0):
        return None

    B = math.atan2(s31, c31)
    cB, sB = math.cos(B), math.sin(B)
    C = (DK[13] - DK[21] * sB) + DK[19] * cB
    ms = DK[21] * cB + DK[19] * sB
    q13_idx_0 = q1 - DK[2]
    k2 = math.atan2(C * k1 - ms * k2, C * k2 + ms * k1) + (-DK[8] - math.pi / 2.0)
    q13_idx_2 = B - DK[14]
    bb_div_cc = joints_approx[3] * math.pi / 180.0 + DK[20]      # J4 estimate incl. theta4
    q1 = q13_idx_0 + DK[2]
    B = k2 + DK[8]
    C = q13_idx_2 + DK[14]
    ms = B + C
    s31 = math.cos(ms)
    c31 = math.cos(q1)
    Hout[0] = s31 * c31
    ai = math.sin(q1)
    Hout[4] = s31 * ai
    ms = math.sin(ms)
    Hout[8] = -ms
    Hout[12] = (DK[3] * ms - DK[7] * s31) - DK[13] * math.cos(C)
    Hout[1] = -math.sin(B + C) * c31
    Hout[5] = -math.sin(B + C) * ai
    Hout[9] = -s31
    Hout[13] = (DK[3] * s31 + DK[7] * ms) + DK[13] * math.sin(C)
    Hout[2] = -ai
    Hout[6] = c31
    Hout[10] = 0.0
    Hout[14] = 0.0
    Hout[3] = Hout[7] = Hout[11] = 0.0
    Hout[15] = 1.0
    for i0 in range(4):
        for i in range(4):
            i1 = i << 2
            base[i0 + i1] = (Hout[i0] * tool[i1] + Hout[i0 + 4] * tool[1 + i1]
                             + Hout[i0 + 8] * tool[2 + i1] + Hout[i0 + 12] * tool[3 + i1])

    # |sin J5| from the two off-axis terms (exact; 1 - cos^2 loses all digits near 0)
    s5 = math.sqrt(base[8] * base[8] + base[10] * base[10])
    if s5 < IK_SING_SIN_EPS:
        # wrist singularity: J4 and J6 axes aligned, keep J4 at the estimate
        C = 0.0 if base[9] >= 0.0 else math.pi
        sn, cs = math.sin(bb_div_cc), math.cos(bb_div_cc)
        ms = math.atan2(sn * base[0] + cs * base[2], sn * base[2] - cs * base[0])
    elif joints_approx[4] >= 0.0:
        bb_div_cc = math.atan2(base[10] / s5, -base[8] / s5)
        C = math.atan2(s5, base[9])
        ms = math.sin(C)
        ms = math.atan2(base[5] / ms, -base[1] / ms)
    else:
        bb_div_cc = math.atan2(-base[10] / s5, base[8] / s5)
        C = math.atan2(-s5, base[9])
        ms = math.sin(C)
        ms = math.atan2(base[5] / ms, -base[1] / ms)

    joints = [q13_idx_0, k2, q13_idx_2, bb_div_cc - DK[20], C - DK[26], ms + (-DK[32] + math.pi)]
    if joints[5] > math.pi:
        joints[5] -= 2.0 * math.pi
    elif joints[5] <= -math.pi:
        joints[5] += 2.0 * math.pi
    return [DK[60 + i] * (joints[i] * 180.0 / math.pi) for i in range(6)]


def _ik_pose(target_pose, joints_estimate):
    """IK including the (identity) user base/tool frames; joints in degrees or None."""
    return _ik_raw(target_pose, _DATA, joints_estimate)


def inverse_kinematics(target_xyzuvw, estimate):
    """Rotation vector in radians (like forward_kinematics output). Joints in degrees or []."""
    if len(target_xyzuvw) != 6:
        raise RuntimeError("Expected 6-element xyzuvw input")
    if len(estimate) != 6:
        raise RuntimeError("Expected 6-element joint estimate")
    j = _ik_pose(_xyzuvw_2_pose([float(v) for v in target_xyzuvw]), [float(v) for v in estimate])
    return j if j is not None else [0.0] * 6


def inverse_kinematics_no_estimate(target_xyzuvw):
    return inverse_kinematics(target_xyzuvw, [0.0] * 6)


def _joints_valid(joints):
    for i in range(ROBOT_nDOFs):
        if joints[i] < -_DATA[_LOWER + i] or joints[i] > _DATA[_UPPER + i]:
            return False
    return True


def _wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def _angdiff(a, b):
    return _wrap180(a - b)


def _sgn_db(v, db):
    if v > db:
        return 1
    if v < -db:
        return -1
    return 0


def _unwrap_into_limits(a, ref, lower_mag, upper):
    """Among a, a+360, a-360 pick the value inside the joint limits closest to ref."""
    best, best_d = a, abs(a - ref)
    for c in (a + 360.0, a - 360.0):
        if c < -lower_mag or c > upper:
            continue
        d = abs(c - ref)
        if d < best_d:
            best, best_d = c, d
    return best


def SolveInverseKinematicsConfigured(xyzuvw_In, JangleIn_in, wrist_config="A", max_jump_deg=0.0):
    """Pose [x, y, z, u, v, w] (mm, rotation vector in DEGREES) + current joints (deg) -> joints.

    wrist_config: "F" (J5 >= 0), "N" (J5 <= 0), "A" automatic (keep the current branch).
    max_jump_deg > 0 rejects a solution that moves any joint more than that far from the
    estimate: for interpolated motion (Move L / Move A / live jogs) consecutive targets are
    ~1 mm apart, so a joint jumping 180 or 360 degrees is a discontinuity, not a move.
    Returns [] (and sets KinematicError) when there is no acceptable solution."""
    global KinematicError
    KinematicError = 0
    if len(xyzuvw_In) != 6:
        raise RuntimeError("Expected 6-element xyzuvw input")
    if len(JangleIn_in) != ROBOT_nDOFs:
        raise RuntimeError("Expected 6-element joint estimate")

    JangleIn = [float(v) for v in JangleIn_in]
    target = [float(xyzuvw_In[0]), float(xyzuvw_In[1]), float(xyzuvw_In[2]),
              math.radians(float(xyzuvw_In[3])), math.radians(float(xyzuvw_In[4])),
              math.radians(float(xyzuvw_In[5]))]
    pose = _xyzuvw_2_pose(target)

    solutions = []
    for branch in (30.0, -30.0):
        est = list(JangleIn)
        est[4] = branch
        joints = _ik_pose(pose, est)
        if joints is None:
            continue
        joints = [_unwrap_into_limits(_wrap180(joints[j]), JangleIn[j], _DATA[_LOWER + j], _DATA[_UPPER + j])
                  for j in range(ROBOT_nDOFs)]
        if not _joints_valid(joints):
            continue
        if any(all(abs(_angdiff(s[j], joints[j])) <= IK_DUP_TOL_DEG for j in range(ROBOT_nDOFs)) for s in solutions):
            continue
        solutions.append(joints)

    if not solutions:
        KinematicError = 1
        return []

    wc = (wrist_config or "")[:1].upper()
    J5_DB, SING_DB = 0.5, 2.0
    cur_wrist_sum = JangleIn[3] + JangleIn[5]
    desired = _sgn_db(JangleIn[4], J5_DB) or 1
    if wc == 'F':
        desired = 1
    if wc == 'N':
        desired = -1

    def travel(sol):
        return sum(abs(sol[j] - JangleIn[j]) for j in range(ROBOT_nDOFs))

    best, best_cost = None, float("inf")
    for sol in solutions:
        j5 = sol[4]
        abs_j5 = abs(j5)
        w5 = _sgn_db(j5, J5_DB)
        cost = travel(sol)
        if wc in ('F', 'N'):
            if abs_j5 > SING_DB:
                if w5 != 0 and w5 != desired:
                    continue
                if w5 == 0:
                    cost += 50.0
            elif w5 != 0 and w5 != desired:
                cost += 200.0
        elif wc == 'A':
            if abs_j5 > SING_DB and w5 != 0 and w5 != desired:
                cost += 20.0
        else:
            if abs_j5 > SING_DB:
                if w5 != 0 and w5 != desired:
                    continue
            elif w5 != 0 and w5 != desired:
                cost += 200.0
        if abs_j5 <= SING_DB:
            cost += 5.0 * abs(_angdiff(sol[3] + sol[5], cur_wrist_sum))
        if cost < best_cost:
            best, best_cost = sol, cost

    if best is None:
        best = min(solutions, key=travel)

    if max_jump_deg > 0.0 and any(abs(best[j] - JangleIn[j]) > max_jump_deg for j in range(ROBOT_nDOFs)):
        KinematicError = 1
        return []

    return list(best)


def SolveInverseKinematics(xyzuvw_In, JangleIn_in, max_jump_deg=0.0):
    return SolveInverseKinematicsConfigured(xyzuvw_In, JangleIn_in, "A", max_jump_deg)
