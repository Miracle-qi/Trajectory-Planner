"""
Linear MPC controller (X-Y frame)
author: huiming zhou
"""

import os
import sys
import math
import cvxpy
import numpy as np
import bisect
import matplotlib.pyplot as plt

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../../MotionPlanning/")

import Simulator.vehicle as vehicle

class MPC:
    # System config
    NX = 4  # state vector: z = [x, y, v, phi]
    NU = 2  # input vector: u = [acceleration, steer]

    dist_stop = 1.5  # stop permitted when dist to goal < dist_stop
    speed_stop = 0.5 / 3.6  # stop permitted when speed < speed_stop
    time_max = 500.0  # max simulation time
    iter_max = 5  # max iteration
    target_speed = 10.0 / 3.6  # target speed
    N_IND = 10  # search index number
    dt = 0.2  # time step
    d_dist = 1.0  # dist step
    du_res = 0.1  # threshold for stopping iteration

    # MPC config
    T = 10  # finite time horizon length
    Q = np.diag([5.0, 5.0, 100.0, 1.0])  # penalty for states
    Qf = np.diag([1.0, 1.0, 1.0, 1.0])  # penalty for end state
    R = np.diag([0.01, 0.1])  # penalty for inputs
    Rd = np.diag([0.01, 1.0])  # penalty for change of inputs


def calc_index(rx, ry, s_t):
    dx = np.diff(rx)
    dy = np.diff(ry)
    ds = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    s = [0]
    s.extend(np.cumsum(ds))
    return bisect.bisect(s, s_t) - 1


def calc_ref_trajectory_in_T_step(car, ref_x, ref_y, ref_yaw, sp_coe):
    """
    calc referent trajectory in T steps: [x, y, v, yaw]
    using the current velocity, calc the T points along the reference path
    :param node: current information
    :param ref_path: reference path: [x, y, yaw]
    :param sp: speed profile (designed speed strategy)
    :return: reference trajectory
    """
    z_ref = np.zeros((MPC.NX, MPC.T + 1))
    length = len(ref_x)

    z_ref[0, 0] = ref_x[0]
    z_ref[1, 0] = ref_y[0]
    z_ref[2, 0] = sp_coe[0]
    z_ref[3, 0] = ref_yaw[0]
    dist_move = 0.0

    for i in range(1, MPC.T + 1):
        t = i * MPC.dt
        v = sp_coe[0] + 2*sp_coe[1]*t + 3*sp_coe[2]*t**2 + 4*sp_coe[3]*t**3 + 5*sp_coe[4]*t**4
        s = sp_coe[0] * t + 2 * sp_coe[1] * t ** 2 + sp_coe[2] * t ** 3 + sp_coe[3] * t ** 4 + sp_coe[4] * t ** 5
        index = calc_index(ref_x, ref_y, s)

        z_ref[0, i] = ref_x[index]
        z_ref[1, i] = ref_y[index]
        z_ref[2, i] = v
        z_ref[3, i] = ref_yaw[index]

    return z_ref


def linear_mpc_control(z_ref, z0, a_old, delta_old):
    """
    linear mpc controller
    :param z_ref: reference trajectory in T steps
    :param z0: initial state vector
    :param a_old: acceleration of T steps of last time
    :param delta_old: delta of T steps of last time
    :return: acceleration and delta strategy based on current information
    """

    if a_old is None or delta_old is None:
        a_old = [0.0] * MPC.T
        delta_old = [0.0] * MPC.T

    x, y, yaw, v = None, None, None, None

    z_bar = predict_states_in_T_step(z0, a_old, delta_old, z_ref)
    a_old, delta_old, x, y, yaw, v = solve_linear_mpc(z_ref, z_bar, z0, delta_old)

    # for k in range(MPC.iter_max):
    #     z_bar = predict_states_in_T_step(z0, a_old, delta_old, z_ref)
    #     a_rec, delta_rec = a_old[:], delta_old[:]
    #     a_old, delta_old, x, y, yaw, v = solve_linear_mpc(z_ref, z_bar, z0, delta_old)
    #
    #     du_a_max = max([abs(ia - iao) for ia, iao in zip(a_old, a_rec)])
    #     du_d_max = max([abs(ide - ido) for ide, ido in zip(delta_old, delta_rec)])
    #
    #     if max(du_a_max, du_d_max) < MPC.du_res:
    #         break

    return a_old, delta_old, x, y, yaw, v


def predict_states_in_T_step(z0, a, delta, z_ref):
    """
    given the current state, using the acceleration and delta strategy of last time,
    predict the states of vehicle in T steps.
    :param z0: initial state
    :param a: acceleration strategy of last time
    :param delta: delta strategy of last time
    :param z_ref: reference trajectory
    :return: predict states in T steps (z_bar, used for calc linear motion model)
    """

    z_bar = z_ref * 0.0

    for i in range(MPC.NX):
        z_bar[i, 0] = z0[i]

    node = vehicle.Vehicle(x=z0[0], y=z0[1], v=z0[2], yaw=z0[3])

    for ai, di, i in zip(a, delta, range(1, MPC.T + 1)):
        node.update(ai, di, 1.0)
        z_bar[0, i] = node.x
        z_bar[1, i] = node.y
        z_bar[2, i] = node.v
        z_bar[3, i] = node.yaw

    return z_bar


def calc_linear_discrete_model(v, phi, delta):
    """
    calc linear and discrete time dynamic model.
    :param v: speed: v_bar
    :param phi: angle of vehicle: phi_bar
    :param delta: steering angle: delta_bar
    :return: A, B, C
    """

    A = np.array([[1.0, 0.0, MPC.dt * math.cos(phi), - MPC.dt * v * math.sin(phi)],
                  [0.0, 1.0, MPC.dt * math.sin(phi), MPC.dt * v * math.cos(phi)],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, MPC.dt * math.tan(delta) / vehicle.Para.WB, 1.0]])

    B = np.array([[0.0, 0.0],
                  [0.0, 0.0],
                  [MPC.dt, 0.0],
                  [0.0, MPC.dt * v / (vehicle.Para.WB * math.cos(delta) ** 2)]])

    C = np.array([MPC.dt * v * math.sin(phi) * phi,
                  -MPC.dt * v * math.cos(phi) * phi,
                  0.0,
                  -MPC.dt * v * delta / (vehicle.Para.WB * math.cos(delta) ** 2)])

    return A, B, C


def solve_linear_mpc(z_ref, z_bar, z0, d_bar):
    """
    solve the quadratic optimization problem using cvxpy, solver: OSQP
    :param z_ref: reference trajectory (desired trajectory: [x, y, v, yaw])
    :param z_bar: predicted states in T steps
    :param z0: initial state
    :param d_bar: delta_bar
    :return: optimal acceleration and steering strategy
    """

    z = cvxpy.Variable((MPC.NX, MPC.T + 1))
    u = cvxpy.Variable((MPC.NU, MPC.T))

    cost = 0.0
    constrains = []

    for t in range(MPC.T):
        cost += cvxpy.quad_form(u[:, t], MPC.R)
        cost += cvxpy.quad_form(z_ref[:, t] - z[:, t], MPC.Q)

        A, B, C = calc_linear_discrete_model(z_bar[2, t], z_bar[3, t], d_bar[t])

        constrains += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C]

        if t < MPC.T - 1:
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], MPC.Rd)
            constrains += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= vehicle.Para.steer_change_max * MPC.dt]

    # print("ref:", z_ref)
    cost += cvxpy.quad_form(z_ref[:, MPC.T] - z[:, MPC.T], MPC.Qf)

    constrains += [z[:, 0] == z0]
    constrains += [z[2, :] <= vehicle.Para.speed_max]
    constrains += [z[2, :] >= vehicle.Para.speed_min]
    constrains += [cvxpy.abs(u[0, :]) <= vehicle.Para.acceleration_max]
    constrains += [cvxpy.abs(u[1, :]) <= vehicle.Para.steer_max]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
    prob.solve(solver=cvxpy.OSQP)

    a, delta, x, y, yaw, v = None, None, None, None, None, None

    if prob.status == cvxpy.OPTIMAL or \
            prob.status == cvxpy.OPTIMAL_INACCURATE:
        x = z.value[0, :]
        y = z.value[1, :]
        v = z.value[2, :]
        yaw = z.value[3, :]
        a = u.value[0, :]
        delta = u.value[1, :]
    else:
        print("Cannot solve linear mpc!")

    return a, delta, x, y, yaw, v


