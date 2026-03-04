import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE

        l = self.arm_length
        gamma = self.k_drag / self.k_thrust

        self.lgamma_mat = np.array([
            [0, l, 0, -l],
            [-l, 0, l, 0],
            [gamma, -gamma, gamma, -gamma],
            [1, 1, 1, 1]
        ])

        # self.K_d = np.array([
        #     [5.3, 0, 0],
        #     [0, 5.3, 0],
        #     [0, 0, 5.3]
        # ])

        # self.K_d = np.array([
        #     [4.9, 0, 0],
        #     [0, 4.9, 0],
        #     [0, 0, 4.9]
        # ])

        # self.K_p = np.array([
        #     [7.75, 0, 0],
        #     [0,7.75, 0],
        #     [0, 0, 7.75]
        # ])


        # controls pass

        # self.K_d = np.array([
        #     [4.9, 0, 0],
        #     [0, 4.9, 0],
        #     [0, 0, 4.9]
        # ])

        # self.K_p = np.array([
        #     [8.75, 0, 0],
        #     [0,8.75, 0],
        #     [0, 0, 8.75]
        # ])


        # self.K_d = np.array([
        #     [6.9, 0, 0],
        #     [0, 7.5, 0],
        #     [0, 0, 6.2]
        # ])


        # fastest speed

        # self.K_d = np.array([
        #     [5.9, 0, 0],
        #     [0, 5.9, 0],
        #     [0, 0, 3.9]
        # ])


        # self.K_p = np.array([
        #     [8.75, 0, 0],
        #     [0,8.75, 0],
        #     [0, 0, 8.75]
        # ])


        # best results so far

        # self.K_d = np.array([
        #     [5.9, 0, 0],
        #     [0, 5.9, 0],
        #     [0, 0, 3.9]
        # ])


        # self.K_p = np.array([
        #     [8.75, 0, 0],
        #     [0,8.75, 0],
        #     [0, 0, 8.75]
        # ])


        #experimenting

        # self.K_d = np.array([
        #     [6.3, 0, 0],
        #     [0, 6.3, 0],
        #     [0, 0, 4.3]
        # ])


        self.K_d = np.array([
            [6.2, 0, 0],
            [0, 6.2, 0],
            [0, 0, 4.0]
        ])


        # self.K_d = np.array([
        #     [5.5, 0, 0],
        #     [0, 5.5, 0],
        #     [0, 0, 5.8]
        # ])

        self.K_d = np.array([
            [5.5, 0, 0],
            [0, 5.5, 0],
            [0, 0, 5.8]
        ])

        # best tuning grd
        # self.K_d = np.array([
        #     [5.9, 0, 0],
        #     [0, 5.9, 0],
        #     [0, 0, 4.9]
        # ])

        # self.K_d = np.array([
        #     [7.4, 0, 0],
        #     [0, 7.4, 0],
        #     [0, 0, 11.75]
        # ])

        # self.K_d = np.array([
        #     [5.6, 0, 0],
        #     [0, 5.6, 0],
        #     [0, 0, 5.3]
        # ])

        # self.K_d = np.array([
        #     [5.4, 0, 0],
        #     [0, 5.4, 0],
        #     [0, 0, 6.5]
        # ])



        self.K_d = np.array([
            [5.4, 0, 0],
            [0, 5.4, 0],
            [0, 0, 6.1]
        ])


        # self.K_d = np.array([
        #     [4.9, 0, 0],
        #     [0, 4.9, 0],
        #     [0, 0, 7.1]
        # ])

    
        # self.K_d = np.array([
        #     [6.3, 0, 0],
        #     [0, 6.3, 0],
        #     [0, 0, 4.7]
        # ])

        # best tuning grd
        # self.K_p = np.array([
        #     [9.0, 0, 0],
        #     [0,9.0, 0],
        #     [0, 0, 8.75]
        # ])

        self.K_p = np.array([
            [14.8, 0, 0],
            [0,14.8, 0],
            [0, 0, 14.8]
        ])


        self.K_d = np.array([
            [5.4, 0, 0],
            [0, 5.4, 0],
            [0, 0, 6.1]
        ])

        # self.K_d = np.array([
        #     [5.6, 0, 0],
        #     [0, 5.6, 0],
        #     [0, 0, 5.8]
        # ])

        self.K_p = np.array([
            [9.0, 0, 0],
            [0,9.0, 0],
            [0, 0, 9.0]
        ])

        # self.K_p = np.array([
        #     [9.5, 0, 0],
        #     [0,9.5, 0],
        #     [0, 0, 9.5]
        # ])

        # self.K_p = np.array([
        #     [9.0, 0, 0],
        #     [0,9.0, 0],
        #     [0, 0, 9.2]
        # ])


        # self.K_p = np.array([
        #     [5.95, 0, 0],
        #     [0,5.95, 0],
        #     [0, 0, 6.55]
        # ])

        # self.K_p = np.array([
        #     [13.05, 0, 0],
        #     [0,13.05, 0],
        #     [0, 0, 13.05]
        # ])



        # self.K_p = np.array([
        #     [9.0, 0, 0],
        #     [0,9.0, 0],
        #     [0, 0, 9.0]
        # ])

        # self.K_p = np.array([
        #     [9.6, 0, 0],
        #     [0,9.6, 0],
        #     [0, 0, 6.9]
        # ])




        # all three pass

        # self.K_d = np.array([
        #     [5.9, 0, 0],
        #     [0, 6.1, 0],
        #     [0, 0, 4.2]
        # ])


        # self.K_p = np.array([
        #     [8.37, 0, 0],
        #     [0,8.37, 0],
        #     [0, 0, 8.75]
        # ])


        # self.K_d = np.array([
        #     [5.75, 0, 0],
        #     [0, 5.75, 0],
        #     [0, 0, 5.75]
        # ])


        # self.K_p = np.array([
        #     [8.75, 0, 0],
        #     [0,8.75, 0],
        #     [0, 0, 8.75]
        # ])


        # self.K_d = np.array([
        #     [5.2, 0, 0],
        #     [0, 5.2, 0],
        #     [0, 0, 5.2]
        # ])

        # self.K_p = np.array([
        #     [9.1, 0, 0],
        #     [0,9.1, 0],
        #     [0, 0, 9.1]
        # ])


        # self.K_d = np.array([
        #     [6.9, 0, 0],
        #     [0, 6.9, 0],
        #     [0, 0, 6.9]
        # ])

        # self.K_p = np.array([
        #     [15.75, 0, 0],
        #     [0,15.75, 0],
        #     [0, 0, 15.75]
        # ])


        # self.K_R = np.array([
        #     [3050.0, 0, 0],
        #     [0, 3050.0, 0],
        #     [0, 0, 250.0]
        # ])

        # self.K_omega = np.array([
        #     [252.0, 0, 0],
        #     [0, 252.0, 0],
        #     [0, 0, 152.0]
        # ])



        # self.K_R = np.array([
        #     [2700.0, 0, 0],
        #     [0, 2700.0, 0],
        #     [0, 0, 180.0]
        # ])

        # self.K_omega = np.array([
        #     [220.0, 0, 0],
        #     [0, 220.0, 0],
        #     [0, 0, 120.0]
        # ])


        # correct KR

        # self.K_R = np.array([
        #     [3050.0, 0, 0],
        #     [0, 3050.0, 0],
        #     [0, 0, 350.0]
        # ])

        # self.K_omega = np.array([
        #     [292.0, 0, 0],
        #     [0, 292.0, 0],
        #     [0, 0, 192.0]
        # ])


        #correct
        # self.K_R = np.array([
        #     [3550.0, 0, 0],
        #     [0, 3550.0, 0],
        #     [0, 0, 350.0]
        # ])

        # self.K_omega = np.array([
        #     [292.0, 0, 0],
        #     [0, 292.0, 0],
        #     [0, 0, 192.0]
        # ])


        # self.K_R = np.array([
        #     [3490.0, 0, 0],
        #     [0, 3490.0, 0],
        #     [0, 0, 310.0]
        # ])

        # self.K_omega = np.array([
        #     [310.0, 0, 0],
        #     [0, 310.0, 0],
        #     [0, 0, 201.0]
        # ])


        # best tuning on gradescope

        # self.K_R = np.array([
        #     [3490.0, 0, 0],
        #     [0, 3490.0, 0],
        #     [0, 0, 260.0]
        # ])

        # self.K_R = np.array([
        #     [3350.0, 0, 0],
        #     [0, 3350.0, 0],
        #     [0, 0, 260.0]
        # ])

        # self.K_omega = np.array([
        #     [345.0, 0, 0],
        #     [0, 345.0, 0],
        #     [0, 0, 248.0]
        # ])


        # CORRECT
        # self.K_R = np.array([
        #     [3350.0, 0, 0],
        #     [0, 3350.0, 0],
        #     [0, 0, 260.0]
        # ])

        # self.K_omega = np.array([
        #     [345.0, 0, 0],
        #     [0, 345.0, 0],
        #     [0, 0, 248.0]
        # ])


        # CORRECT
        # self.K_R = np.array([
        #     [3350.0, 0, 0],
        #     [0, 3350.0, 0],
        #     [0, 0, 260.0]
        # ])

        # self.K_omega = np.array([
        #     [345.0, 0, 0],
        #     [0, 345.0, 0],
        #     [0, 0, 248.0]
        # ])

        self.K_R = np.array([
            [3265.0, 0, 0],
            [0, 3265.0, 0],
            [0, 0, 277.0]
        ])

        self.K_omega = np.array([
            [362.0, 0, 0],
            [0, 362.0, 0],
            [0, 0, 148.0]
        ])

        # self.K_omega = np.array([
        #     [362.0, 0, 0],
        #     [0, 362.0, 0],
        #     [0, 0, 258.0]
        # ])

        # self.K_R = np.array([
        #     [3195.0, 0, 0],
        #     [0, 3195.0, 0],
        #     [0, 0, 277.0]
        # ])

        # self.K_omega = np.array([
        #     [240.0, 0, 0],
        #     [0, 240.0, 0],
        #     [0, 0, 258.0]
        # ])

        # MAJOR EXPERIMENT

        # self.K_R = np.array([
        #     [3080.0, 0, 0],
        #     [0, 3080.0, 0],
        #     [0, 0, 237.0]
        # ])


        # # MAJOR EXPERIMENT
        # self.K_R = np.array([
        #     [2810.0, 0, 0],
        #     [0, 2810.0, 0],
        #     [0, 0, 237.0]
        # ])

        # self.K_omega = np.array([
        #     [362.0, 0, 0],
        #     [0, 362.0, 0],
        #     [0, 0, 258.0]
        # ])


        # MAJOR EXPERIMENT
        # self.K_omega = np.array([
        #     [312.0, 0, 0],
        #     [0, 312.0, 0],
        #     [0, 0, 158.0]
        # ])



        # original tuning
        # self.K_R = np.array([
        #     [3220.0, 0, 0],
        #     [0, 3220.0, 0],
        #     [0, 0, 265.0]
        # ])

        # self.K_omega = np.array([
        #     [380.0, 0, 0],
        #     [0, 380.0, 0],
        #     [0, 0, 182.0]
        # ])




        # testing
        # self.K_R = np.array([
        #     [3590.0, 0, 0],
        #     [0, 3590.0, 0],
        #     [0, 0, 390.0]
        # ])

        # self.K_omega = np.array([
        #     [297.0, 0, 0],
        #     [0, 297.0, 0],
        #     [0, 0, 197.0]
        # ])

        # self.Kd = 0.025
        # self.Kd = 0.01
        # self.Kd = 0.012
        # self.Kd = 0.0270
        self.Kd = 0.0135
        # self.Kd = 0.0310
        # self.Kd = 0.0215
        # self.Kd = 0.0136

        # self.K_R = np.array([
        #     [3750.0, 0, 0],
        #     [0, 3750.0, 0],
        #     [0, 0, 370.0]
        # ])

        # self.K_omega = np.array([
        #     [295.0, 0, 0],
        #     [0, 295.0, 0],
        #     [0, 0, 195.0]
        # ])

        

        # retuning

        # self.K_d = np.array([
        #     [1.0, 0, 0],
        #     [0, 1.0, 0],
        #     [0, 0, 1.0]
        # ])

        # self.K_p = np.array([
        #     [2.75, 0, 0],
        #     [0, 2.75, 0],
        #     [0, 0, 2.75]
        # ])


        # self.K_R = np.array([
        #     [2050.0, 0, 0],
        #     [0, 2050.0, 0],
        #     [0, 0, 150.0]
        # ])

        # self.K_omega = np.array([
        #     [152.0, 0, 0],
        #     [0, 152.0, 0],
        #     [0, 0, 52.0]
        # ])


    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # print("desired: ", flat_output)
        # print('\n')
        # print("present state: ", state)
        # print('\n')

        
        def so3_2_R3(V):
            # print("v:")
            # print(V)
            wx = V[2, 1]
            wy = V[0, 2]
            wz = V[1, 0]

            # print(wx)

            return np.array([wx,wy,wz])

        r = state['x']
        r_dot = state['v']

        r_T = flat_output['x']
        r_dot_T = flat_output['x_dot']
        r_ddot_T = flat_output['x_ddot']
        r_dddot_T = flat_output['x_dddot']

        kdm = (self.Kd * r_dot)



        # r_ddot_T = 0.7 * r_ddot_T + (1 - 0.7) * 

        # if np.linalg.norm(r_dot - r_dot_T) > 0.3:
        #     r_dot_T = r_dot

        # print("rddotT: ", r_dddot_T.shape)

        #original
        # r_ddot_des = r_ddot_T - self.K_d @ (r_dot - r_ddot_T) - self.K_p @ (r - r_T)
        r_ddot_des = r_ddot_T - self.K_d @ (r_dot - r_dot_T) - self.K_p @ (r - r_T)

        # print("r_dot_shape", r_ddot_des.shape)
        # print('desired acc. ', r_ddot_des)

        F_des = (self.mass * r_ddot_des) + np.array([0, 0, self.mass * self.g]).T

        R_conv = Rotation.from_quat(state['q'])
        R = R_conv.as_matrix()
        # print("R shape ", R.shape)
        b3 = R @ np.array([0, 0, 1]).T

        # print("b3 shape", b3)
        
        u_1 = b3.T @ F_des
        # print("u_1", u_1)

        # print("F_des", F_des.shape)


        # eps_des = 0.5
        # norm_acc = np.linalg.norm()



        # b3_des = F_des/(np.linalg.norm(F_des))
        b3_des = (F_des - kdm)/(np.linalg.norm(F_des - kdm))

        b3_dot = np.dot(b3, b3_des)
        ang_change = np.arccos(b3_dot)

        # if ang_change > 0.50:
        #     b3_des = (b3 + b3_des) / 2
        #     b3_des /= np.linalg.norm(b3_des)

        # b3 = b3_des.copy()

        # if ang_change > 0.35:
        #     b3_des = (b3 + b3_des) / 2
        #     b3_des /= np.linalg.norm(b3_des)

        # b3 = b3_des.copy()

        if ang_change > 0.350:
            b3_des = (b3 + b3_des) / 2
            b3_des /= np.linalg.norm(b3_des)
            # sin_ = np.sin(ang_change)

        b3 = b3_des.copy()


        # if ang_change > 0.26:
        #     b3_des = (b3 + b3_des)
        #     b3_des /= np.linalg.norm(b3_des)

        # b3 = b3_des.copy()


        # if ang_change > 0.26:
        #     b3_des = (b3 + b3_des) / 2
        #     b3_des /= np.linalg.norm(b3_des)

        # b3 = b3_des.copy()




        # print("b3 shape", b3_des)
        psi_T = flat_output['yaw']
        # a_psi = np.array([np.sin(psi_T), np.cos(psi_T), 0]).T
        a_psi = np.array([np.cos(psi_T), np.sin(psi_T), 0]).T

        b2_des = np.cross(b3_des, a_psi) / np.linalg.norm(np.cross(b3_des, a_psi))

        # print("b2 shape:", b2_des.shape)

        # R_des = np.array([np.cross(b2_des, b3_des), b2_des, b3_des])
        R_des = np.column_stack([np.cross(b2_des, b3_des), b2_des, b3_des])

        # print("R_des: ", R_des.shape)

        e_R = 0.5 * so3_2_R3(R_des.T @ R - R.T @ R_des)
        # print("shape e_R imm: ", e_R.shape)
        # print(state['w'].shape)
        e_w = state['w'] - flat_output['yaw_dot']

        u_2 = self.inertia @ ((- 1 * self.K_R @ e_R) - (self.K_omega @ e_w))


        # print("shape e_R: ", e_R.shape)
        # print("shape e_w: ", e_w.shape)
        # print("K_r mult e_R", (- 1 * self.K_R @ e_R).shape)
        # print("shape K_omega", K_ome)


        # print("u_1: ", u_1.shape)
        # print("u_2: ", u_2.shape)
        # print(u_1[2])
        # cmd_thrust = u_1[2]
        # cmd_moment = u_2
        # print(u_1)


        # r = Rotation.from_quat

        # psi_ = state.q[3]
        # a_psi = np.array([np.cos(psi_[3]), np.sin(psi_[3], 0)])


        # b2_des = np.cross()

        cmd_thrust = u_1
        cmd_moment = u_2

        # u = np.append(u_1, u_2)
        u = np.append(u_2, u_1)
        # print("u: ", u)

        motor_forces = np.linalg.solve(self.lgamma_mat, u)

        # print(motor_forces)

        rotor_speeds = np.sqrt(np.maximum((motor_forces / self.k_thrust), 0.0))
        # print(rotor_speeds)
        cmd_motor_speeds = np.clip(rotor_speeds,self.rotor_speed_min, self.rotor_speed_max)

        cmd_q = Rotation.from_matrix(R_des).as_quat()

        # STUDENT CODE HERE

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input
