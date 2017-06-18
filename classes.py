import numpy as np
from matplotlib import pyplot as plt


class Kinematics:
    def __init__(self, T, N):
        self.N = N
        self.del_t = T / (N - 1)

    def v(self, x):
        v = np.zeros(self.N - 1)
        v = (x[1:] - x[:-1]) / self.del_t
        return v

    def a(self, x):
        v_ = self.v(x)

        a = np.zeros(self.N - 2)
        a = (v_[1:] - v_[:-1]) / self.del_t
        return a

    def j(self, x):
        a_ = self.a(x)

        j = np.zeros(self.N - 3)
        j = (a_[1:] - a_[:-1]) / self.del_t
        return j


class CostFunction(Kinematics):
    def __init__(self, T, N, v_des, wv, wa, wj):
        super().__init__(T, N)
        self.v_des = v_des
        self.del_t = T / (N - 1)
        self.wv = wv
        self.wa = wa
        self.wj = wj

    def cost(self, x):
        return 0.5 * (self.wv * sum((self.v(x) - self.v_des) ** 2) + self.wa * sum(self.a(x) ** 2)
                      + self.wj * sum(self.j(x) ** 2)) * self.del_t


class PhysicalConstraints(Kinematics):
    def __init__(self, T, N, v_max, a_max, j_max):
        super().__init__(T, N)
        self.del_t = T / (N - 1)
        self.a_max = a_max
        self.j_max = j_max
        self.v_max = v_max

    def v_pos_cons(self, x):
        return self.v(x)

    def v_cons(self, x):
        return self.v_max - self.v(x)

    def a_cons(self, x):
        return self.a_max - self.a(x)

    def j_cons(self, x):
        return self.j_max - self.j(x)


class ObstacleConstraints(Kinematics):
    def __init__(self, N, T, vel_init, acc_init):
        super().__init__(T, N)
        self.vel_init = vel_init
        self.acc_init = acc_init
        self.N = N
        self.T = T
        self.del_t = T/(N - 1)

    def bounds_initial(self):
        bounds = [(0, np.inf) for _ in range(self.N)]
        return bounds

    def bounds_first_points(self, bounds):
        bounds[0] = (0, 0)
        bounds[1] = (self.vel_init * self.del_t, self.vel_init * self.del_t)
        bounds[2] = (self.acc_init * self.del_t ** 2 + 2 * self.vel_init * self.del_t,
                     self.acc_init * self.del_t ** 2 + 2 * self.vel_init * self.del_t)
        return bounds

    def crossing_obstacle(self, t_i, t_j, x_i, x_j):
        t_i_n = int(t_i//self.del_t)
        t_j_n = int(-(-t_j//self.del_t))

        bounds = [self.bounds_initial(), self.bounds_initial()]

        for i in range(3, t_j_n + 1):
            bounds[0][i] = (0, (x_i//1))

        for j in range(t_i_n, self.N):
            bounds[1][j] = (-(-x_j//1), np.inf)

        bounds[0] = self.bounds_first_points(bounds[0])
        bounds[1] = self.bounds_first_points(bounds[1])

        return bounds

    def compare(self, r0, r1):
        shell = np.zeros((len(r0), 2))
        bounds = [(0, 0) for _ in range(len(r0))]

        for i in range(len(r0)):

            if r0[i][0] < r1[i][0]:
                shell[i][0] = r1[i][0]

            else:
                shell[i][0] = r0[i][0]

            if r0[i][1] > r1[i][1]:
                shell[i][1] = r1[i][1]

            else:
                shell[i][1] = r0[i][1]

        for i in range(len(r0)):
            bounds[i] = (shell[i][0], shell[i][1])

        return bounds

    def expand(self, r0, r1):
        return self.compare(r0, r1[0]), self.compare(r0, r1[1])

    def filter_func(self, bounds):
        i = True
        x = 0

        while x < self.N and i:
            if bounds[x][0] > bounds[x][1]:
                i = False
            x += 1

        return i

    def obstacles(self, r):
        obs_numb = len(r)
        bounds_ = [i for i in range(obs_numb)]

        for i in range(obs_numb):
            bounds_[i] = self.crossing_obstacle(*r[i])

        new_list = bounds_[0]

        if obs_numb > 1:
            for j in range(1, obs_numb):
                bound_list = [i for i in range(2*len(new_list))]

                for i in range(0, len(new_list)):
                    bound_list[2*i], bound_list[2*i+1] = self.expand(new_list[i], bounds_[j])

                new_list = list(filter(self.filter_func, bound_list))

        return new_list


class Plots:
    def __init__(self, T, N):
        self.T = T
        self.N = N
        self.del_t = T/(N - 1)

    def x_axis(self):
        t = np.arange(0, self.T + self.del_t, self.del_t)
        return t

    def crossing_obstacles(self, t_i, t_j, x_i, x_j):

        t_i_n = (t_i//self.del_t)*self.del_t
        t_j_n = -(-t_j//self.del_t)*self.del_t

        x = np.array([t_i_n, t_j_n])
        y1 = np.ones_like(x)*(-1)*(-x_j//1)
        y2 = np.ones_like(x)*(x_i//1)

        plt.fill_between(x, y1, y2)
