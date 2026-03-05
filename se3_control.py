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



        self.K_d = np.array([
            [1.4, 0, 0],
            [0, 1.4, 0],
            [0, 0, 0.1]
        ])


        self.K_p = np.array([
            [1.0, 0, 0],
            [0,0.0, 0],
            [0, 0, 9.0]
        ])

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

        self.Kd = 0.0135


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
        r_ddot_des = r_ddot_T - self.K_d @ (r_dot - r_dot_T) - self.K_p @ (r - r_T)


        F_des = (self.mass * r_ddot_des) + np.array([0, 0, self.mass * self.g]).T

        R_conv = Rotation.from_quat(state['q'])
        R = R_conv.as_matrix()
        # print("R shape ", R.shape)
        b3 = R @ np.array([0, 0, 1]).T
        
        u_1 = b3.T @ F_des
        b3_des = -1 * (F_des + kdm)/(np.linalg.norm(F_des - kdm))

        b3_dot = np.dot(b3, b3_des)
        ang_change = np.arccos(b3_dot)


        if ang_change > 0.350:
            b3_des = (b3 + b3_des) / 2
            b3_des /= np.linalg.norm(b3_des)
            # sin_ = np.sin(ang_change)

        b3 = b3_des.copy()


        psi_T = flat_output['yaw']
        a_psi = np.array([np.cos(psi_T), np.sin(psi_T), 0])

        b2_des = np.cross(b3_des, -a_psi) / np.linalg.norm(np.cross(b3_des, a_psi))

        R_des = np.column_stack([np.cross(-1*b2_des, b3_des), b2_des, -b3_des])

        e_R = 0.5 * so3_2_R3(R_des.T @ R + R.T @ R_des)
        e_w = state['w'] - flat_output['yaw_dot']

        u_2 = self.inertia @ ((- 1 * self.K_R @ e_R) + (self.K_omega @ e_w))

        cmd_thrust = u_1
        cmd_moment = u_2

        u = np.append(u_2, u_1)

        motor_forces = np.linalg.solve(self.lgamma_mat, u)

        rotor_speeds = np.sqrt(np.maximum((motor_forces / self.k_thrust), 0.0))
        cmd_motor_speeds = np.clip(rotor_speeds,self.rotor_speed_min, self.rotor_speed_max)

        cmd_q = Rotation.from_matrix(R_des).as_quat()

        # STUDENT CODE HERE

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input
