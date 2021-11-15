import math

class Traj_Point:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.x_dot = 0
        self.y_dot = 0
        self.theta = 0
        self.velocity = 0
        self.acceleration = 0
        self.curvature = 0
        self.sumdistance = 0


class Path:
    def __init__(self):
        self.t = []

        self.l = []
        self.l_v = []
        self.l_a = []
        self.l_jerk = []

        self.s = []
        self.s_v = []
        self.s_a = []
        self.s_jerk = []

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.curv = []

        self.cost = 0.0

def calc_yaw_curv(x, y):
    yaw, curv, ds = [], [], []

    for i in range(len(x) - 1):
        dx = x[i + 1] - x[i]
        dy = y[i + 1] - y[i]
        ds.append(math.hypot(dx, dy))
        yaw.append(math.atan2(dy, dx))

    if len(yaw) == 0:
        return None, None, None

    yaw.append(yaw[-1])
    ds.append(ds[-1])

    for i in range(len(yaw) - 1):
        curv.append((yaw[i + 1] - yaw[i]) / ds[i])

    return yaw, curv, ds


def is_path_collision(path):
    index = range(0, len(path.x), 5)
    x = [path.x[i] for i in index]
    y = [path.y[i] for i in index]
    yaw = [path.yaw[i] for i in index]

    for ix, iy, iyaw in zip(x, y, yaw):
        d = 1.8
        dl = (C.RF - C.RB) / 2.0
        r = math.hypot((C.RF + C.RB) / 2.0, C.W / 2.0) + d

        cx = ix + dl * math.cos(iyaw)
        cy = iy + dl * math.sin(iyaw)

        for i in range(len(C.obs)):
            xo = C.obs[i][0] - cx
            yo = C.obs[i][1] - cy
            dx = xo * math.cos(iyaw) + yo * math.sin(iyaw)
            dy = -xo * math.sin(iyaw) + yo * math.cos(iyaw)

            if abs(dx) < r and abs(dy) < C.W / 2 + d:
                return 1.0

    return 0.0


def pi_2_pi(theta):
    while theta > math.pi or theta < -math.pi:

        if theta > math.pi:
            theta -= 2.0 * math.pi

        if theta < -math.pi:
            theta += 2.0 * math.pi

    return theta

def verify_path(path):
    # if any([v > C.speed_max for v in path.s_v]) or \
    #         any([abs(a) > C.acceleration_max for a in path.s_a]):
    #     return False

    if any([v > C.MAX_SPEED for v in path.s_v]) or \
            any([abs(a) > C.MAX_ACCEL for a in path.s_a]) or \
            any([abs(curv) > C.MAX_CURVATURE for curv in path.curv]):
        return False

    return True


class PATH:
    def __init__(self, cx, cy, cyaw, ck):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.length = len(cx)
        self.ind_old = 0

    def nearest_index(self, node):
        """
        calc index of the nearest node in N steps
        :param node: current information
        :return: nearest index, lateral distance to ref point
        """

        dx = [node.x - x for x in self.cx[self.ind_old: (self.ind_old + P.N_IND)]]
        dy = [node.y - y for y in self.cy[self.ind_old: (self.ind_old + P.N_IND)]]
        dist = np.hypot(dx, dy)

        ind_in_N = int(np.argmin(dist))
        ind = self.ind_old + ind_in_N
        self.ind_old = ind

        rear_axle_vec_rot_90 = np.array([[math.cos(node.yaw + math.pi / 2.0)],
                                         [math.sin(node.yaw + math.pi / 2.0)]])

        vec_target_2_rear = np.array([[dx[ind_in_N]],
                                      [dy[ind_in_N]]])

        er = np.dot(vec_target_2_rear.T, rear_axle_vec_rot_90)
        er = er[0][0]

        return ind, er