import math
import numpy as np
from CurvesGenerator import cubic_spline
import common

class Para:

    # parameters for vehicle
    K_SIZE = 1.4
    RF = 4.5 * K_SIZE  # [m] distance from rear to vehicle front end of vehicle
    RB = 1.0 * K_SIZE  # [m] distance from rear to vehicle back end of vehicle
    W = 3.0 * K_SIZE  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 3.5 * K_SIZE  # [m] Wheel base
    TR = 0.5 * K_SIZE  # [m] Tyre radius
    TW = 1 * K_SIZE  # [m] Tyre width

    steer_max = np.deg2rad(45.0)  # max steering angle [rad]
    steer_change_max = np.deg2rad(30.0)  # maximum steering speed [rad/s]
    speed_max = 30  # maximum speed [m/s]
    speed_min = -20.0 # minimum speed [m/s]
    acceleration_max = 20.0  # maximum acceleration [m/s2]



# todo: add limitation on acc and steer_velocity
class Vehicle:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, a=0.0, dt=0.1, direct=1.0, status="following", lane=1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.acc = a
        self.direct = direct
        self.dt = dt
        self.status = status
        self.laneIndex = lane

    def update(self, a, delta, direct):
        delta = self.limit_input_delta(delta)
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.v / Para.WB * math.tan(delta) * self.dt
        self.direct = direct
        self.v += self.direct * a * self.dt
        self.v = self.limit_speed(self.v)

    @staticmethod
    def limit_input_delta(delta):
        if delta >= Para.steer_max:
            return Para.steer_max

        if delta <= -Para.steer_max:
            return -Para.steer_max

        return delta

    @staticmethod
    def limit_speed(v):
        if v >= Para.speed_max:
            return Para.speed_max

        if v <= Para.speed_min:
            return Para.speed_min

        return v

def get_nearest_waypoints(ref_x, ref_y, cur_x, cur_y, cur_yaw):
    # Find the nearest forward node
    i, i_nearest = 0, 0
    min_dis = 20
    while i < len(ref_x):
        i_dir = [ref_x[i] - cur_x, ref_y[i] - cur_y]
        i_dir_norm = math.sqrt(i_dir[0]**2+i_dir[1]**2)

        if i_dir_norm < min_dis:
            theta = math.atan2(i_dir[1], i_dir[0])
            delta = abs(common.pi_2_pi(theta - cur_yaw))
            if delta < math.pi / 2:
                min_dis = i_dir_norm
                i_nearest = i
        i += 1
    return i_nearest