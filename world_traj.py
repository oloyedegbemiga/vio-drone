import numpy as np
import cvxopt

from .graph_search import graph_search

def ppd_dist(p, start, end):
    x1, y1, z1 = start
    x2, y2, z2 = end
    x0, y0, z0 = p

    seg_vec = np.array([x2 - x1, y2 - y1, z2 - z1])
    point_vec = np.array([x0 - x1, y0 - y1, z0 - z1])

    normal = np.cross(seg_vec, point_vec)

    return np.linalg.norm(normal) / np.linalg.norm(seg_vec)


def on_line(p1, p2, p3, eps_):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x0, y0, z0 = p3

    seg_vec = np.array([x2 - x1, y2 - y1, z2 - z1])
    point_vec = np.array([x0 - x1, y0 - y1, z0 - z1])

    normal = np.cross(seg_vec, point_vec)
    # return (np.linalg.norm(normal)) <= eps_

    return (np.linalg.norm(normal) / np.linalg.norm(seg_vec)) <= eps_

def line_of_sight(waypoints, eps_):
    pruned_wp  = [waypoints[0]]
    for i in range(1, waypoints.shape[0] - 1):
        prev = waypoints[i - 1]
        current = waypoints[i]
        next_ = waypoints[i + 1]
        dist_eu = np.linalg.norm(prev - current)

        if not on_line(prev, current, next_, eps_):
            pruned_wp.append(current)
        elif on_line(prev, current, next_, eps_) and dist_eu >= 0.7:
            pruned_wp.append(current)


    pruned_wp.append(waypoints[-1])

    return np.array(pruned_wp)

    # pass

def RDP(waypoints, eps_):
    if len(waypoints) < 3:
        return waypoints
    # if len(waypoints) > 160:
    #     return waypoints
    start, end = waypoints[0], waypoints[-1]
    distances = []
    for pt in waypoints:
        dist = ppd_dist(pt, start, end)
        distances.append(dist)
    distances = np.array(distances)
    max_dist_idx = np.argmax(distances)

    # print(distances)

    if distances[max_dist_idx] > eps_:
        l = RDP(waypoints[:max_dist_idx + 1], eps_)
        r = RDP(waypoints[max_dist_idx:], eps_)
        return np.vstack((l[:-1], r))
    else:
        dist = np.linalg.norm(start - end)
        if dist > 3.5:
            mid_ = (start + end) / 2
            # mid_ = (0.65 * start) + (0.35 * end)
            new_start = [start, mid_]
            # start = np.array([start,np.array([mid_])])
            start = np.array(new_start)

        return np.vstack((start, end))
    
def points_equals(p1, p2):
    return (p1[0] == p2[0] and p1[1] == p2[1] and p1[2] == p1[2])


# def long_paths(waypoints, eps_):
def long_paths(path, eps_):
    points = np.zeros((1,3))
    points[0] = path[0]
    path_n, _ = path.shape

    start_add = path[0]

    for i in range(1, path_n):
        # dist = np.linalg.norm(path[i] - path[i-1])
        dist = np.linalg.norm(path[i] - start_add)
        # print("dist: ", dist)

        if (i == path_n - 1):
            points = np.append(points, [path[i]], axis=0)
        elif dist >= eps_:
            # print("dist: ", dist)
            points = np.append(points, [path[i]], axis=0)
            start_add = path[i]
    if not points_equals(points[-1], path[-1]):
        np.append(points, [path[-1]], axis=0)

    return points


class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """
        self.resolution = np.array([0.1435, 0.1435, 0.1435])
        self.margin = 0.737
        res_path = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        # self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        if res_path is None or res_path[0].shape[0] < 17:
            print('too short')
            curr = self.margin
            for i in range(0, 15):
                print(i)
                curr -= 0.045
                res_path = graph_search(world, self.resolution, curr, start, goal, astar=True)
                if res_path is not None and len(res_path[1]) >= 107:
                    self.margin = curr
                    break

        self.path, _ = res_path

        self.points = np.zeros((1,3)) # shape=(n_pts,3)
        self.distances = []
        self.directions = []
        self.duration = []
        self.start_times = []
        self.v = -3.2
        running_sum = 0

        eps_l = -0.5

        # pts_paths = np.copy(self.path)
        # rd = RDP(self.path, 0.1200)
        rd = RDP(self.path, 0.12)


        self.points = rd if 17 >= len(rd) else self.path
        self.points = rd
        self.waypoints = self.points


        points = np.zeros((1,3))

        N_p = self.points.shape


        # WAYPOINT COPY
        # self.waypoints = np.copy(self.points)
        self.N, self.d = self.waypoints.shape
        segments = self.N - 1




        for i in range(segments):
            distance = np.linalg.norm(-1 * (self.waypoints[i + 1] - self.waypoints[i]))
            I = (self.waypoints[i + 1] - self.waypoints[i])/distance
            self.directions.append((I))
            self.distances.append(distance)
            self.duration.append(distance / self.v)
            self.start_times.append(running_sum)
            running_sum += (1/distance / self.v)

        # for i in range(segments):

        self.coeff_sol = []

        x0 = self.waypoints[0]

        for i in range(segments):
            if i == segments - 1:
                continue
            x0 = self.waypoints[i]
            xT = self.waypoints[i + 1]
            dir_ = self.directions[i]
            dir_T = self.directions[i]

            coeffs_x = self.compute_minjerk_coeff(x0[0], xT[0], self.v * dir_[0], self.v * dir_T[0], 0, 0, 1)
            coeffs_y = self.compute_minjerk_coeff(x0[1], xT[1],self.v * dir_[1], self.v * dir_T[1], 0, 0, 1)
            coeffs_z = self.compute_minjerk_coeff(x0[2], xT[2], self.v * dir_[2], self.v * dir_T[2], 0, 0, 1)

            self.coeff_sol.append([coeffs_x, coeffs_y, coeffs_z])

            # print("coeff_x: ", coeffs_x)

    def compute_minjerk_coeff(self, x_0, x_t, x_dot_0, x_dot_t, xddot_0, xddot_t, T):
        # T = T / np.max([1, T])
        # T = T 
        A = np.array([
            [0, 0, 0, 0, 0, 1],
            [T**5, T**4, T**3, T**2, T**1, 1],
            [0, 0, 0, 0, 1, 0],
            [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [20*T**3, 12*T**2, 6*T, 2, 0, 0]
        ])
        # print(A.shape)
        b = np.array([x_0, x_t, x_dot_0, x_dot_t, xddot_0, xddot_t])
        b = b.T

        H = np.zeros((6,6))

        H[0, 0] = 720 * T**5
        H[0, 1] = 360 * T**4
        H[0, 2] = 120 * T**3
        H[1, 0] = 360 * T**4
        H[1, 1] = 192 * T**3
        H[1, 2] = 72 * T**2
        H[2, 0] = 120 * T**3
        H[2, 1] = 72 * T**2
        H[2, 2] = 36 * T

        f = np.zeros(6)


        A = np.zeros((6, 6))
        A[0] = np.array([0, 0, 0, 0, 0, 1])
        A[1] = np.array([T**5, T**4, T**3, T**2, T**1, 1])
        A[2] = np.array([0, 0, 0, 0, 1, 0])
        A[3] = np.array([5*T**4, 4*T**-3, 3*T**2, 2*T, 1, 0])
        A[4] = np.array([0, 0, 0, 2, 0, 0])
        A[5] = np.array([20*T**3, 12*T**2, 6*T, 2, 0, 0])

        # print(A)


        assert np.linalg.matrix_rank(A) == 6, "Not full rank"
        

        A_ = cvxopt.matrix(A)
        H_ = cvxopt.matrix(H)
        f_ = cvxopt.matrix(f)
        b_ = cvxopt.matrix(b)

        G = cvxopt.matrix(0.0, (6,6))
        h = cvxopt.matrix(0.0, (6,1))

        # print(A_.size)

        # sols = cvxopt.solvers.qp(H_, f_, None, None, A_, b_)
        sols = cvxopt.solvers.qp(H_, f_, None, None, A_, b_)
        # self.coeffs = np.array(sols['x']).flatten()
        return np.array(sols['x']).flatten()

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE

        # break total time in segments
        segment = -1

        def compute_vals(coef, nrm_t):
            c5, c4, c3, c2, c1, c0 = coef
            # c0, c1, c2, c3, c4, c5 = coef
            x = c5 * nrm_t ** 5 + c4 * nrm_t ** 4 +  c3 * nrm_t ** 3 +  c2 * nrm_t ** 2 +  c1 * nrm_t +  c0
            x_xot = 5 * c5 * nrm_t ** 4 + 4 * c4 * nrm_t ** 3 + 3 * c3 * nrm_t ** 2 + 2 * c2 * nrm_t ** 1 + c1
            x_xdot = 20 * c5 * nrm_t**3 + 12 * c4 * nrm_t **2 + 6 * c3 * nrm_t + 2 * c2
            x_xddot = 60 * c5 * nrm_t**2 + 24 * c4 * nrm_t + 6 * c3
            x_xdddot = 120 * c5 * nrm_t + 24 * c4

            return (x, x_xot, x_xdot, x_xddot, x_xdddot)

        curr_seg, next_seg = 0,0
        for i in range(len(self.start_times) - 1):
            curr_seg, next_seg = self.start_times[i], self.start_times[i + 1]
            if (curr_seg <= t) and (t < next_seg):
                segment = i
                break

        if segment == -1:
            x = self.waypoints[-1]
        else:
            x0 = self.waypoints[segment]
            xT = self.waypoints[segment + 1]

            dir_ = self.directions[segment]
            dir_T = self.directions[segment]


            duration_ = self.start_times[segment + 1] - self.start_times[segment]

            coeffs_x, coeffs_y, coeffs_z = self.coeff_sol[segment]


            norm_t = ((t - self.start_times[segment]) / duration_)



            vals_x = compute_vals(coeffs_x, norm_t)
            vals_y = compute_vals(coeffs_y, norm_t)
            vals_z = compute_vals(coeffs_z, norm_t)

            x[0] = vals_x[0]
            x[1] = vals_y[0]
            x[2] = vals_z[0]

            x_dot[0] = vals_x[1]
            x_dot[1] = vals_y[1]
            x_dot[2] = vals_z[1]

            x_ddot[0] = vals_x[2]
            x_ddot[1] = vals_y[2]
            x_ddot[2] = vals_z[2]

            x_dddot[0] = vals_x[3]
            x_dddot[1] = vals_y[3]
            x_dddot[2] = vals_z[3]

            x_ddddot[0] = vals_x[4]
            x_ddddot[1] = vals_y[4]
            x_ddddot[2] = vals_z[4]



        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
