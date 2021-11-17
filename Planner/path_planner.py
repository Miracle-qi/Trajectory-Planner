from CurvesGenerator import cubic_spline
import numpy as np


def following_path(wp_x, wp_y, d_dist):
    cubicspline = cubic_spline.Spline2D(wp_x, wp_y)
    s = np.arange(0, cubicspline.s[-1], d_dist)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = cubicspline.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(cubicspline.calc_yaw(i_s))
        rk.append(cubicspline.calc_curvature(i_s))
    return rx, ry, ryaw, rk, cubicspline