####################################################################################################
# Imports:
####################################################################################################

import numpy as np

from math import atan2, asin, cos, sin, sqrt

####################################################################################################
# Functions:
####################################################################################################

def eulToR(eul):
    cos_yaw = cos(eul[2])
    cos_pitch = cos(eul[1])
    cos_roll = cos(eul[0])
    sin_yaw = sin(eul[2])
    sin_pitch = sin(eul[1])
    sin_roll = sin(eul[0])

    mat = np.ndarray((3,3))

    mat[0,0] = cos_pitch*cos_yaw
    mat[0,1] = sin_roll*sin_pitch*cos_yaw-cos_roll*sin_yaw
    mat[0,2] = cos_roll*sin_pitch*cos_yaw+sin_roll*sin_yaw
    mat[1,0] = cos_pitch*sin_yaw
    mat[1,1] = sin_roll*sin_pitch*sin_yaw+cos_roll*cos_yaw
    mat[1,2] = cos_roll*sin_pitch*sin_yaw-sin_roll*cos_pitch
    mat[2,0] = -sin_pitch
    mat[2,1] = sin_roll*cos_pitch
    mat[2,2] = cos_roll*cos_pitch

    return mat

def quatToEul(quat):
    x = atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]), 1-2*(quat[1]*quat[1] + quat[2]*quat[2])),
    y = asin(2*(quat[0]*quat[2] - quat[3]*quat[1])),
    z = atan2(2*(quat[0]*quat[3] + quat[1]*quat[2]), 1-2*(quat[2]*quat[2]+quat[3]*quat[3]))

    return np.array([x, y, z])

def quatInv(quat):
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]])

def quatMultiply(quat1, quat2):
    w = quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
    x = quat1[0]*quat2[1] + quat1[1]*quat2[0] + quat1[2]*quat2[3] - quat1[3]*quat2[2],
    y = quat1[0]*quat2[2] - quat1[1]*quat2[3] + quat1[2]*quat2[0] + quat1[3]*quat2[1],
    z = quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1] + quat1[3]*quat2[0]

    return np.array([w, x, y, z])

def quatToMat(quat):
    mat = np.ndarray((3,3))

    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3) 
    r02 = 2 * (q1 * q3 + q0 * q2) 
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    mat[0,0] = r00
    mat[0,1] = r01
    mat[0,2] = r02
    mat[1,0] = r10
    mat[1,1] = r11
    mat[1,2] = r12
    mat[2,0] = r20
    mat[2,1] = r21
    mat[2,2] = r22

    return mat

def matToQuat(R):
    tr = R[0,0] + R[1,1] + R[2,2]

    qw, qx, qy, qz = 0,0,0,0

    if (tr > 0):

        S = sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S

    elif ((R[0,0] > R[1,1]) and (R[0,0] > R[2,2])):
        S = sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S 
        qz = (R[0,2] + R[2,0]) / S 

    elif (R(1,1) > R(2,2)):
        S = sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S 
    else:
        S = sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S

    quat = [qw, qx, qy, qz]

    return quat

def eulToQuat(eul):
    roll, pitch, yaw = eul[0], eul[1], eul[2]
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return np.array([qw, qx, qy, qz])
