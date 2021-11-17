import math
import common
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
import Simulator.vehicle as car
from sympy import *
from CurvesGenerator import cubic_spline, quintic_polynomial, quartic_polynomial


def speed_profile(cx, cy, cyaw, target_speed):
    """
    design appropriate speed strategy
    :param cx: x of reference path [m]
    :param cy: y of reference path [m]
    :param cyaw: yaw of reference path [m]
    :param target_speed: target speed [m/s]
    :return: speed profile
    """
    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(common.pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def speed_profile_quinticPoly(vehicle, vel_goal, acc_goal, path):
    s_sum = path.s[-1]

    vel_start = vehicle.v
    acc_start = vehicle.acc
    vel_limit = car.Para.speed_max
    acc_limit = car.Para.acceleration_max
    T = 2 * s_sum/(vel_goal+vel_start)
    seg = 10
    Q = [0.1, 0.005, 5, 0.1]
    delta_t = T/seg

    a = cp.Variable(5)
    c_1 = [T, T ** 2, T ** 3, T ** 4, T ** 5]
    c_2 = [1, 2 * T, 3 * T ** 2, 4 * T ** 3, 5 * T ** 4]
    c_3 = [1, 0, 0, 0, 0]
    c_4 = [0, 2, 6*T, 12*T**2, 20*T**3]
    c_5 = [0, 2, 0, 0, 0]
    c_0, c_6, c_7 = [], [], []
    for t in np.arange(delta_t, T, delta_t):
        c_0.append([0, 0, 6, 24 * t, 60 * t ** 2])
        c_6.append([1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4])
        c_7.append([0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3])
    c_0 = np.matrix(c_0)
    c_6 = np.matrix(c_6)
    c_7 = np.matrix(c_7)

    N = len(np.arange(delta_t, T, delta_t))
    objective = cp.Minimize(Q[0] * cp.sum_squares(c_0 @ a) + Q[2] * cp.square(cp.sum(a @ c_2)
                            - vel_goal) + Q[3] * cp.square(cp.sum(a @ c_4) - acc_goal))
    constraints = [cp.sum(a @ c_1) == s_sum,
                   cp.sum(a @ c_3) == vel_start,
                   cp.sum(a @ c_5) == acc_start,
                   c_6 @ a >= np.zeros(N),
                   c_6 @ a <= np.full(N, vel_limit),
                   c_7 @ a <= np.full(N, acc_limit),
                   c_7 @ a >= -np.full(N, acc_limit)]

    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.SCS)
    print(prob.status)
    print(a.value)

    return a.value


if __name__ == '__main__':
    vel_limit = 5.0
    vel_goal =  3.0
    vel_start = 0.0
    acc_limit = 10.0
    acc_start = 0.0
    acc_goal = 0.0
    T = 3/(vel_goal+vel_start)
    seg = 10
    Q = [0.1, 0.005, 5, 10]
    delta_t = T/seg

    a = cp.Variable(5)
    c_1 = [T, T ** 2, T ** 3, T ** 4, T ** 5]
    c_2 = [1, 2 * T, 3 * T ** 2, 4 * T ** 3, 5 * T ** 4]
    c_3 = [1, 0, 0, 0, 0]
    c_4 = [0, 2, 6*T, 12*T**2, 20*T**3]
    c_5 = [0, 2, 0, 0, 0]
    c_0, c_6, c_7 = [], [], []
    for t in np.arange(delta_t, T, delta_t):
        c_0.append([0, 0, 6, 24 * t, 60 * t ** 2])
        c_6.append([1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4])
        c_7.append([0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3])
    c_0 = np.matrix(c_0)
    c_6 = np.matrix(c_6)
    c_7 = np.matrix(c_7)

    objective = cp.Minimize(Q[2] * cp.square(cp.sum(a @ c_2) - vel_goal) + Q[3] * cp.square(cp.sum(a @ c_4) - acc_goal))
    constraints = [cp.sum(a @ c_1) == 1,
                   a[0] == vel_start,
                   cp.sum(a @ c_5) == acc_start,
                   c_6 @ a >= np.zeros(seg-1),
                   c_6 @ a <= np.full(seg-1, vel_limit),
                   c_7 @ a <= np.full(seg-1, acc_limit),
                   c_7 @ a >= -np.full(seg-1, acc_limit)]

    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.SCS)
    print(prob.status)
    print(a.value)

    s, v, acc = [], [], []
    ts = []
    for t in np.arange(0., T+0.01, 0.01):
        ts.append(t)
        s.append(a.value[0]*t + a.value[1]*t**2 + a.value[2]*t**3 + a.value[3]*t**4 + a.value[4]*t**5)
        v.append(a.value[0] + 2 * a.value[1] * t + 3 * a.value[2] * t ** 2 + 4 * a.value[3] * t ** 3 + 5 * a.value[4] * t ** 4)
        acc.append(2 * a.value[1] + 6 * a.value[2] * t + 12 * a.value[3] * t ** 2 + 20 * a.value[4] * t ** 3)
    plt.plot(ts, s, "r")
    plt.figure()
    plt.plot(ts, v)
    plt.figure()
    plt.plot(ts, acc)
    plt.show()
