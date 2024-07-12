import os
from math import cos, sin, sqrt, asin, atan2


def AttitudeSolu(pitch, roll, yaw, gyr_x, gyr_y, gyr_z):
    w = [0]*3
    q = [0]*4
    dq = [0]*4
    dt = 0.005
    w[0] = gyr_x * 0.017452
    w[1] = gyr_y * 0.017452
    w[2] = gyr_z * 0.017452
    p = pitch * 0.017452
    r = roll * 0.017452
    y = yaw * 0.017452
    T_11 = cos(r) * cos(y) - sin(r) * sin(p) * sin(y)
    T_21 = cos(r) * sin(y) + sin(r) * sin(p) * cos(y)
    T_31 = -sin(r) * cos(p)
    T_12 = -cos(p) * sin(y)
    T_22 = cos(p) * cos(y)
    T_32 = sin(p)
    T_13 = sin(r) * cos(y) + cos(r) * sin(p) * sin(y)
    T_23 = sin(r) * sin(y) - cos(r) * sin(p) * cos(y)
    T_33 = cos(r) * cos(p)
    q[0] = 0.5 * sqrt(1 + T_11 + T_22 + T_33)
    q[1] = 0.5 * sqrt(1 + T_11 - T_22 - T_33)
    q[2] = 0.5 * sqrt(1 - T_11 + T_22 - T_33)
    q[3] = 0.5 * sqrt(1 - T_11 - T_22 + T_33)
    if (T_32 - T_23) < 0:
        q[1] = -q[1]
    if (T_13 - T_31) < 0:
        q[2] = -q[2]
    if (T_21 - T_12) < 0:
        q[3] = -q[3]
    dq[0] = 0.5 * (-w[0] * q[1] - w[1] * q[2] - w[2] * q[3])
    dq[1] = 0.5 * (w[0] * q[0] + w[2] * q[2] - w[1] * q[3])
    dq[2] = 0.5 * (w[1] * q[0] - w[2] * q[1] + w[0] * q[3])
    dq[3] = 0.5 * (w[2] * q[0] + w[1] * q[1] - w[0] * q[2])
    q[0] = q[0] + dq[0] * dt
    q[1] = q[1] + dq[1] * dt
    q[2] = q[2] + dq[2] * dt
    q[3] = q[3] + dq[3] * dt
    q_norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    q[0] = q[0] / q_norm
    q[1] = q[1] / q_norm
    q[2] = q[2] / q_norm
    q[3] = q[3] / q_norm
    p = asin(2 * (q[0] * q[1] + q[2] * q[3])) * 57.3
    y = atan2(-2 * (q[1] * q[2] - q[0] * q[3]),
              (pow(q[0], 2) - pow(q[1], 2) + pow(q[2], 2) - pow(q[3], 2))) * 57.3
    r = atan2(-2 * (q[1] * q[3] - q[0] * q[2]),
              pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2) + pow(q[3], 2)) * 57.3
    if r < -90:
        r = 180 + r
        if p > 0:
            p = 180 - p
        else:
            p = -180 - p
        if y > 0:
            y = y - 180
        else:
            y = 180 + y
    if r > 90:
        r = r - 180
        if p > 0:
            p = 180 - p
        else:
            p = -180 - p
    if y > 0:
        y = y - 180
    else:
        y = 180 + y
    return p, r, y

if __name__ == '__main__':
    path = '../data.txt'
    flag = 1
    with open(path, 'r') as f:
        for line in f:
            splitline = line.split(' ')
            if flag == 1:
                pitch = float(splitline[18])
                roll = float(splitline[20])
                yaw = float(splitline[22])
                flag = 0
            gx = float(splitline[6])
            gy = float(splitline[8])
            gz = float(splitline[10])
            p, r, y = AttitudeSolu(pitch,roll,yaw,gx,gy,gz)
            pitch = p
            roll = r
            yaw = y
            with open('data.txt','a') as datafile:
                str = '%f %f %f\n' %(p,r,y)
                datafile.write(str)

