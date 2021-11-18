from CurvesGenerator import cubic_spline, Bezier_curve
import numpy as np
import Simulator.vehicle as vehicle

def following_path(rx, ry, car, d_dist):

    i_nearest = vehicle.get_nearest_waypoints(rx, ry, car.x, car.y, car.yaw)
    index = range(i_nearest, i_nearest+6, 1)
    wp_x = [rx[n] for n in index]
    wp_y = [ry[n] for n in index]

    if wp_x[0] != car.x:
        wp_x.insert(0, car.x)
        wp_y.insert(0, car.y)

    cubicspline = cubic_spline.Spline2D(wp_x, wp_y)
    s = np.arange(0, cubicspline.s[-1], d_dist)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = cubicspline.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(cubicspline.calc_yaw(i_s))
        rk.append(cubicspline.calc_curvature(i_s))

    return rx, ry, ryaw, cubicspline.s[-1]


def Path_Planner(car, rx_1, ry_1, rx_2, ry_2):
    print(car.status)
    if(car.status == "following"):
        if(car.laneIndex == 1):
            i_min = vehicle.get_nearest_waypoints(rx_1, ry_1, car.x, car.y, car.yaw)
            print(i_min)
            if (i_min == 25):
                car.status = "laneChanging"
            rx, ry, ryaw, s_sum = following_path(rx_1, ry_1, car, 0.1)
        else:
            rx, ry, ryaw, s_sum = following_path(rx_2, ry_2, car, 0.1)
    else: # lane changing
        if(np.hypot(car.x - rx_2[30], car.y - ry_2[30]) < 2):
            car.status = "following"
            car.laneIndex = 2
        rx, ry, ryaw, s_sum = laneChanging_path(car, 25, rx_2, ry_2)
    return rx, ry, ryaw, s_sum


def laneChanging_path(car, i_min, rx_2, ry_2):
    # generate reference line
    index = range(i_min+5, i_min+10, 1)
    new_wp_x = [rx_2[i] for i in index]
    new_wp_y = [ry_2[i] for i in index]

    new_cubicspline = cubic_spline.Spline2D(new_wp_x, new_wp_y)
    s = np.arange(0, new_cubicspline.s[-1], 0.1)
    new_rx, new_ry, new_ryaw = [], [], []
    for i_s in s:
        ix, iy = new_cubicspline.calc_position(i_s)
        new_rx.append(ix)
        new_ry.append(iy)
        new_ryaw.append(new_cubicspline.calc_yaw(i_s))
    rx, ry, ryaw, s = Bezier_curve.calc_path(car.x, car.y, car.yaw, new_rx[0], new_ry[0], new_ryaw[0], 3, 0.1)
    # rx.extend(new_rx)
    # ry.extend(new_ry)
    # ryaw.extend(new_ryaw)

    return rx, ry, ryaw, s[-1]