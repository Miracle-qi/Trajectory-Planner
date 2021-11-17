import os
import sys
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../MotionPlanning/")

from CurvesGenerator import cubic_spline, quintic_polynomial, quartic_polynomial
import Simulator.env as env
import Simulator.draw as draw
import Simulator.vehicle as vehicle
import Controller.linear_MPC as mpc
import Planner.path_planner as p_planner
import Planner.velocity_planner as v_planner
import common


def main_Crusing():

    # Initialize simulator
    ENV = env.ENVCrusing()
    road_width = 8
    rx_1, ry_1 = ENV.design_reference_line(0)
    rx_1.extend(rx_1[0:10])
    ry_1.extend(ry_1[0:10])
    rx_2, ry_2 = ENV.design_reference_line(road_width)
    bx1, by1 = ENV.design_boundary_in(road_width)
    bx2, by2 = ENV.design_boundary_out(road_width*2)
    # Initialize vehicle
    car = vehicle.Vehicle(x=rx_1[0], y=ry_1[0], yaw=-1, v=0.0, a=0.0, dt=mpc.MPC.dt)
    delta_opt, a_opt = None, None
    a_exc, delta_exc = 0.0, 0.0

    vel_target, acc_goal = 10.0, 0.0
    fig = plt.figure()
    im = []

    while True:
        # Generate reference path
        old_yaw = car.yaw
        wp_x, wp_y = vehicle.get_local_waypoints(rx_1, ry_1, car.x, car.y, car.yaw, 20)
        if wp_x[0] != car.x:
            wp_x.insert(0, car.x)
            wp_y.insert(0, car.y)
        rx, ry, ryaw, rk, cubicspline = p_planner.following_path(wp_x, wp_y, mpc.MPC.d_dist)

        # Generate speed profile
        poly_coe = v_planner.speed_profile_quinticPoly(car, vel_target, acc_goal, cubicspline)

        # Generate reference state for MPC
        z_ref = mpc.calc_ref_trajectory_in_T_step(car, rx, ry, ryaw, poly_coe)

        # MPC controller
        z0 = [car.x, car.y, car.v, car.yaw]
        a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = mpc.linear_mpc_control(z_ref, z0, a_opt, delta_opt)

        # Update simulation
        if delta_opt is not None:
            delta_exc, a_exc = delta_opt[0], a_opt[0]
        car.update(a_exc, delta_exc, 1.0)
        print(a_exc, delta_exc)

        dy = (car.yaw - old_yaw) / (car.v * mpc.MPC.dt)
        steer = common.pi_2_pi(-math.atan(vehicle.Para.WB * dy))
        # Update Visualization

        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(rx_1, ry_1, linestyle='--', color='gray')
        plt.plot(rx_2, ry_2, linestyle='--', color='gray')
        plt.plot(rx, ry, linestyle='--', color='r')
        plt.plot(bx1, by1, linewidth=1.5, color='k')
        plt.plot(bx2, by2, linewidth=1.5, color='k')
        plt.plot(wp_x, wp_y, 'oy')
        # plt.plot(path.x[1:], path.y[1:], linewidth='2', color='royalblue')
        # plt.plot(obs_x, obs_y, 'ok')
        draw.draw_car(car.x, car.y, car.yaw, steer, C=vehicle.Para)
        # plt.title("[Crusing Mode]  v :" + str(s0_v * 3.6)[0:4] + " km/h")
        # plt.axis("equal")
        plt.pause(0.0001)
    plt.show()



if __name__ == '__main__':
    main_Crusing()

