"""
Environment for Lattice Planner Simulation
"""

import math
import numpy as np
import matplotlib.pyplot as plt


class ENVCrusing:
    def __init__(self):
        self.max_c = 0.15
        self.road_width = 8.0
        # self.ref_line = self.design_reference_line()
        # self.bound_in = self.design_boundary_in()
        # self.bound_out = self.design_boundary_out()

    @staticmethod
    def design_reference_line(cr_delta=0):
        rx, ry, ryaw, rc = [], [], [], []
        step_curve = 0.1 * math.pi
        step_line = 4


        cx, cy, cr = 30, 30, 20+cr_delta
        theta = np.arange(math.pi, math.pi * 1.5, step_curve)
        for itheta in theta:
            rx.append(cx + cr * math.cos(itheta))
            ry.append(cy + cr * math.sin(itheta))

        for ix in np.arange(30, 80, step_line):
            rx.append(ix)
            ry.append(10-cr_delta)

        cx, cy, cr = 80, 25, 15+cr_delta
        theta = np.arange(-math.pi / 2.0, math.pi / 2.0, step_curve)
        for itheta in theta:
            rx.append(cx + cr * math.cos(itheta))
            ry.append(cy + cr * math.sin(itheta))

        for ix in np.arange(80, 60, -step_line):
            rx.append(ix)
            ry.append(40+cr_delta)

        cx, cy, cr = 60, 60, 20-cr_delta
        theta = np.arange(-math.pi / 2.0, -math.pi, -step_curve)
        for itheta in theta:
            rx.append(cx + cr * math.cos(itheta))
            ry.append(cy + cr * math.sin(itheta))

        cx, cy, cr = 25, 60, 15+cr_delta
        theta = np.arange(0.0, math.pi, step_curve)
        for itheta in theta:
            rx.append(cx + cr * math.cos(itheta))
            ry.append(cy + cr * math.sin(itheta))

        for iy in np.arange(60, 30, -step_line):
            rx.append(10-cr_delta)
            ry.append(iy)

        return rx, ry

    def design_boundary_in(self, road_width):
        bx, by = [], []
        step_curve = 0.1
        step_line = 2

        cx, cy, cr = 30, 30, 20 - road_width
        theta = np.arange(math.pi, math.pi * 1.5, step_curve)
        for itheta in theta:
            bx.append(cx + cr * math.cos(itheta))
            by.append(cy + cr * math.sin(itheta))

        for ix in np.arange(30, 80, step_line):
            bx.append(ix)
            by.append(10 + road_width)

        cx, cy, cr = 80, 25, 15 - road_width
        theta = np.arange(-math.pi / 2.0, math.pi / 2.0, step_curve)
        for itheta in theta:
            bx.append(cx + cr * math.cos(itheta))
            by.append(cy + cr * math.sin(itheta))

        for ix in np.arange(80, 60, -step_line):
            bx.append(ix)
            by.append(40 - road_width)

        cx, cy, cr = 60, 60, 20 + road_width
        theta = np.arange(-math.pi / 2.0, -math.pi, -step_curve)
        for itheta in theta:
            bx.append(cx + cr * math.cos(itheta))
            by.append(cy + cr * math.sin(itheta))

        cx, cy, cr = 25, 60, 15 - road_width
        theta = np.arange(0.0, math.pi, step_curve)
        for itheta in theta:
            bx.append(cx + cr * math.cos(itheta))
            by.append(cy + cr * math.sin(itheta))

        for iy in np.arange(60, 30, -step_line):
            bx.append(10 + road_width)
            by.append(iy)

        bx.append(bx[0])
        by.append(by[0])

        return bx, by

    def design_boundary_out(self, road_width):
        bx, by = [], []
        step_curve = 0.1
        step_line = 2

        cx, cy, cr = 30, 30, 20 + road_width
        theta = np.arange(math.pi, math.pi * 1.5, step_curve)
        for itheta in theta:
            bx.append(cx + cr * math.cos(itheta))
            by.append(cy + cr * math.sin(itheta))

        for ix in np.arange(30, 80, step_line):
            bx.append(ix)
            by.append(10 - road_width)

        cx, cy, cr = 80, 25, 15 + road_width
        theta = np.arange(-math.pi / 2.0, math.pi / 2.0, step_curve)
        for itheta in theta:
            bx.append(cx + cr * math.cos(itheta))
            by.append(cy + cr * math.sin(itheta))

        for ix in np.arange(80, 60, -step_line):
            bx.append(ix)
            by.append(40 + road_width)

        cx, cy, cr = 60, 60, 20 - road_width
        theta = np.arange(-math.pi / 2.0, -math.pi, -step_curve)
        for itheta in theta:
            bx.append(cx + cr * math.cos(itheta))
            by.append(cy + cr * math.sin(itheta))

        cx, cy, cr = 25, 60, 15 + road_width
        theta = np.arange(0.0, math.pi, step_curve)
        for itheta in theta:
            bx.append(cx + cr * math.cos(itheta))
            by.append(cy + cr * math.sin(itheta))

        for iy in np.arange(60, 30, -step_line):
            bx.append(10 - road_width)
            by.append(iy)

        bx.append(bx[0])
        by.append(by[0])

        return bx, by

class Vehicle_Model:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, direct=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct

    def update(self, a, delta, direct):
        delta = self.limit_input_delta(delta)
        self.x += self.v * math.cos(self.yaw) * P.dt
        self.y += self.v * math.sin(self.yaw) * P.dt
        self.yaw += self.v / P.WB * math.tan(delta) * P.dt
        self.direct = direct
        self.v += self.direct * a * P.dt
        self.v = self.limit_speed(self.v)

    @staticmethod
    def limit_input_delta(delta):
        if delta >= P.steer_max:
            return P.steer_max

        if delta <= -P.steer_max:
            return -P.steer_max

        return delta

    @staticmethod
    def limit_speed(v):
        if v >= P.speed_max:
            return P.speed_max

        if v <= P.speed_min:
            return P.speed_min

        return v


def main():
    env = ENVCrusing()
    road_width = 8
    rx_1, ry_1 = env.design_reference_line(0)
    rx_2, ry_2 = env.design_reference_line(road_width)
    bx1, by1 = env.design_boundary_in(road_width)
    bx2, by2 = env.design_boundary_out(road_width*2)

    plt.plot(rx_1, ry_1, marker='.')
    plt.plot(rx_2, ry_2, marker='.')
    plt.plot(bx1, by1, linewidth=.5, color='k')
    plt.plot(bx2, by2, linewidth=1.5, color='k')
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
