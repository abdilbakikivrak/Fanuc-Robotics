


import numpy as np


# robot dh parameters

A1, A2, A3 = 150.0, 600.0, 200.0
D1, D4 = 450.0, 640.0



# homegenus and  transform

def dh(theta, d, a, alpha):

    c = np.cos(theta)
    s = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [c, -s*ca,  s*sa, a*c],
        [s,  c*ca, -c*sa, a*s],
        [0,  sa,    ca,   d],
        [0,  0,     0,    1]
    ], dtype=float)



# forward kinematics

def FK(q_fanuc_deg):

    q = np.deg2rad(q_fanuc_deg)

    th1 = q[0]
    th2 = q[1]
    th3 = -q[2] - q[1]
    th4 = q[3]
    th5 = -q[4]
    th6 = q[5]

    T01 = dh(th1, D1, A1, -np.pi/2)
    T12 = dh(th2, 0, A2, 0)
    T23 = dh(th3, 0, A3, -np.pi/2)
    T34 = dh(th4, D4, 0, np.pi/2)
    T45 = dh(th5, 0, 0, -np.pi/2)
    T56 = dh(th6, 0, 0, 0)

    T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56

    return T06



# wpr conversion fanuc Rz(W) * Ry(P) * Rx(R)

def rotation_to_wpr(R):

    P = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))

    if abs(np.cos(P)) > 1e-8:

        W = np.arctan2(R[1, 0], R[0, 0])
        R_angle = np.arctan2(R[2, 1], R[2, 2])

    else:

        W = np.arctan2(-R[0, 1], R[1, 1])
        R_angle = 0.0

    return np.rad2deg([W, P, R_angle])


# external axis transformation

def translation_matrix(x=0, y=0, z=0):

    T = np.eye(4)

    T[:3, 3] = [x, y, z]

    return T



#  tool transformation

# eplace this identity matrix with actual
# UT=1 tool transformation

T_FLANGE_TCP = np.eye(4)



#calculate cartesian position

def calculate_cartesian(
    joints,
    E1,
    E2,
    GP2_J1,
    T_EXTERNAL=None,
    T_USER=np.eye(4)
):

    # robot arm fk
    T06 = FK(joints)

    # external axes
    if T_EXTERNAL is None:
        T_EXTERNAL = np.eye(4)

    # World -> external axes -> robot -> tcp
    T_WORLD_TCP = (
        T_EXTERNAL
        @ T06
        @ T_FLANGE_TCP
    )

    # convert into selected user frame
    T_USER_TCP = np.linalg.inv(T_USER) @ T_WORLD_TCP

    # XYZ
    X, Y, Z = T_USER_TCP[:3, 3]

    # WPR
    W, P, R = rotation_to_wpr(T_USER_TCP[:3, :3])

    return {
        "X": X,
        "Y": Y,
        "Z": Z,
        "W": W,
        "P": P,
        "R": R,
        "E1": E1,
        "E2": E2,
        "GP2_J1": GP2_J1
    }



# P[X] joint axis transform test 

joints = [
    -16.341,
    -45.318,
     57.020,
     52.062,
    -28.227,
     28.372
]

E1 = 1975.365
E2 = 103.254
GP2_J1 = -1606.922

# Temporary, external axes are not yet modeled
# This is only the robot-arm coordinate result.

result = calculate_cartesian(
    joints,
    E1,
    E2,
    GP2_J1
)

print("\n transforming cartesian coordinats")

for key, value in result.items():

    print(f"{key:8s} = {value:12.4f}")
