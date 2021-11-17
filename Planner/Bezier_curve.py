import math
import matplotlib.pyplot as plt
from Path_generator.common import Traj_Point



def path_planner_3Dbezier(start_pos, start_dir, goal_pos, goal_dir, interval):

    control_point_shift = 1.0 / 3
    length = math.sqrt((goal_pos[0] - start_pos[0]) ** 2 + (goal_pos[1] - start_pos[1]) ** 2)
    norm_goal_dir = math.sqrt(goal_dir[0]**2 + goal_dir[1]**2)
    goal_dir[0] = goal_dir[0] / norm_goal_dir
    goal_dir[1] = goal_dir[1] / norm_goal_dir
    norm_start_dir = math.sqrt(start_dir[0]**2 + start_dir[1]**2)
    start_dir[0] = start_dir[0] / (norm_start_dir+1e-5)
    start_dir[1] = start_dir[1] / (norm_start_dir+1e-5)
    P1 = [0, 0]
    P2 = [0, 0]
    P1[0] = start_pos[0] + length * control_point_shift * start_dir[0]
    P1[1] = start_pos[1] + length * control_point_shift * start_dir[1]
    P2[0] = goal_pos[0] - length * control_point_shift * goal_dir[0]
    P2[1] = goal_pos[1] - length * control_point_shift * goal_dir[1]
    P0 = start_pos
    P3 = goal_pos
    appro_length = math.sqrt((start_pos[0] - P1[0]) ** 2 + (start_pos[1] - P1[1]) ** 2) + math.sqrt((goal_pos[0] - P2[0]) ** 2 + \
                             (goal_pos[1] - P2[1]) ** 2) + math.sqrt((P1[0] - P2[0]) ** 2 + (P1[1] - P2[1]) ** 2) + \
                   math.sqrt((start_pos[0] - goal_pos[0]) ** 2 + (start_pos[1] - goal_pos[1]) **2)
    appro_length = appro_length * 0.5
    dt = interval / (appro_length + 0.01)
    if (dt < 1e-3):
        dt = 1e-3
    t = 0
    traj_points=[]
    while t < 1:
        point = Traj_Point()
        point.x = P0[0] * (1 - 3 * t + 3 * t * t - t * t * t) + P1[0] * (3 * t - 6 * t * t + 3 * t * t * t) + P2[0] * (3 * t * t - 3 * t * t * t) + P3[0] * (t * t * t)
        point.y = P0[1] * (1 - 3 * t + 3 * t * t - t * t * t) + P1[1] * (3 * t - 6 * t * t + 3 * t * t * t) + P2[1] * (3 * t * t - 3 * t * t * t) + P3[1] * (t * t * t)
        point.x_dot = P0[0] * (-3 + 6 * t - 3 * t * t) + P1[0] * (3 - 12 * t + 9 * t * t) + P2[0] * (6 * t - 9 * t * t) + P3[0] * (3 * t * t)
        point.y_dot = P0[1] * (-3 + 6 * t - 3 * t * t) + P1[1] * (3 - 12 * t + 9 * t * t) + P2[1] * (6 * t - 9 * t * t) + P3[1] * (3 * t * t)
        x_ddot = P0[0] * (6 - 6 * t) + P1[0] * (-12 + 18 * t) + P2[0] * (6 - 18 * t) + P3[0] * (6 * t)
        y_ddot = P0[1] * (6 - 6 * t) + P1[1] * (-12 + 18 * t) + P2[1] * (6 - 18 * t) + P3[1] * (6 * t)
        point.theta = 0
        if (point.x_dot == 0):
            if (point.y_dot > 0):
                point.theta = math.pi / 2
            elif (point.y_dot < 0):
                point.theta = -math.pi / 2
            else:
                point.theta = 0
        else:
            point.theta = math.atan2(point.y_dot, point.x_dot)
        point.curvature = (point.x_dot * y_ddot - x_ddot * point.y_dot) / ((point.x_dot * point.x_dot + point.y_dot * point.y_dot) ** (3.0 / 2.0))
        norm_dir = math.sqrt(point.x_dot ** 2 + point.y_dot ** 2)
        point.x_dot = point.x_dot / norm_dir
        point.y_dot = point.y_dot / norm_dir
        traj_points.append(point)
        t = t + dt
    return traj_points


if __name__ == '__main__':
    traj_test = path_planner_3Dbezier([0, 0], [1, 0], [60, 20], [1, 0],  0.2)
    traj_x, traj_y = [], []
    traj_x_dot, traj_y_dot = [], []
    traj_curv = []
    for p in traj_test:
        traj_x.append(p.x)
        traj_y.append(p.y)
        traj_x_dot.append(p.x_dot)
        traj_y_dot.append(p.y_dot)
        traj_curv.append(p.curvature)
    plt.figure()
    plt.plot(traj_x, traj_y, "r")
    plt.figure()
    plt.plot(traj_x_dot, "g")
    plt.figure()
    plt.plot(traj_y_dot, "o")
    plt.figure()
    plt.plot(traj_curv, "b")
    plt.show()