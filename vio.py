#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                    all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state


    # YOUR CODE HERE

    # R_3 = Rotation.from_quat(q)
    # R_3 = R_3.as_matrix()
    R_3 = q.as_matrix()

    am_minus_ab = (a_m - a_b)

    new_p = p + (v * dt) + 0.5 * ((R_3 @ am_minus_ab) + g) * (dt**2)
    new_v = v + ((R_3 @ am_minus_ab) + g) * dt


    rotvect_quat = ((w_m - w_b) * dt).reshape(3,)
    quat_rot = Rotation.from_rotvec(rotvect_quat)
    new_q = q * quat_rot

    return new_p, new_v, new_q, a_b, w_b, g


def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state


    # YOUR CODE HERE

    def skew_sym(x):
        x_1 = x.reshape(3,)
        return np.array(
            [
                [0.0, -x_1[2], x_1[1]],
                [x_1[2], 0.0, -x_1[0]],
                [-x_1[1], x_1[0], 0.0]
            ]
        )

    Fx = np.zeros((18, 18))
    Fi = np.zeros((18, 12))
    Qi = np.zeros((12, 12))

    am_minus_ab = a_m - a_b

    am_minus_ab_skew = skew_sym(am_minus_ab)

    rotvect_quat = ((w_m - w_b) * dt).reshape(3,)
    quat_rot = Rotation.from_rotvec(rotvect_quat)
    R_wmwb = quat_rot.as_matrix()

    I3 = np.eye(3)
    R_3 = q.as_matrix()



    # Fx
    Fx[0:3, 0:3] = I3
    Fx[0:3, 3:6] = I3 * dt
    Fx[3:6, 3:6] = I3
    Fx[3:6, 6:9] = -R_3 @ am_minus_ab_skew * dt
    Fx[3:6, 9:12] = -R_3 * dt
    Fx[3:6, 15:18] = I3 * dt
    Fx[6:9, 6:9] = R_wmwb.T
    Fx[6:9, 12:15] = -I3 * dt
    Fx[9:12, 9:12] = I3
    Fx[12:15, 12:15] = I3
    Fx[15:18, 15:18] = I3

    #Fi
    Fi[3:6, 0:3] = I3
    Fi[6:9, 3:6] = I3
    Fi[9:12, 6:9] = I3
    Fi[12:15, 9:12] = I3

    #Qi
    Vi = (accelerometer_noise_density ** 2) * (dt**2) * I3
    THETA_i = (gyroscope_noise_density **2) * (dt**2) * I3
    Ai = (accelerometer_random_walk ** 2) * dt * I3
    OMEGA_i =(gyroscope_random_walk ** 2) * dt * I3

    Qi[0:3, 0:3] = Vi
    Qi[3:6, 3:6] = THETA_i
    Qi[6:9, 6:9] = Ai
    Qi[9:12, 9:12] = OMEGA_i

    P = (Fx @ error_state_covariance @ Fx.T) + (Fi @ Qi @ Fi.T) # 18x18 Cov Matrix


    # return an 18x18 covariance matrix
    # return np.identity(18)
    return P


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """
    
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    def skew_sym(x):
        x_1 = x.reshape(3,)
        return np.array(
            [
                [0.0, -x_1[2], x_1[1]],
                [x_1[2], 0.0, -x_1[0]],
                [-x_1[1], x_1[0], 0.0]
            ]
        )
    R_3 = q.as_matrix()
    R_0 = R_3

    P_c = R_3.T @ (Pw - p)
    Z_c = P_c[2, 0]
    X_c = P_c[0, 0]
    Y_c = P_c[1, 0]

    uv_10 = np.array(
        [
            [1.0, 0.0, -X_c / Z_c],
            [0.0, 1.0, -Y_c / Z_c]
        ]
    )

    
    dzt_dP_c = (1/Z_c) * uv_10
    # innovation = uv - ((P_c[0:2])/P_c[2])
    innovation = uv - ((1/Z_c) * ((P_c[0:2])))


    if (np.linalg.norm(innovation) > error_threshold):
        return (p, v, q, a_b, w_b, g), error_state_covariance, innovation

    P_c0 = P_c

    dzt_dtheta = dzt_dP_c @ skew_sym(P_c0)
    dzt_dp = dzt_dP_c @ -R_0.T

    H_t = np.zeros((2, 18))

    H_t[0:2,0:3] = dzt_dp
    H_t[0:2, 6:9] = dzt_dtheta

    I18 = np.eye(18)

    K_t = -1* error_state_covariance @ H_t.T @ np.linalg.inv(H_t @  error_state_covariance @ H_t.T + Q)
    
    I_minus_KtHt = (I18 - (K_t @ H_t))
    Pcov = (I_minus_KtHt) @ error_state_covariance @ (I_minus_KtHt) + (K_t @ Q @ K_t.T)

    delta_x = K_t @ innovation
    dp, dv, dq, da_b, dw_b, dg = [np.array(delta_x[i:i+3]) for i in range(0, 18, 3)]
    p_1, v_1, a_b_1, w_b_1, g_1 = p + dp, v - dv, a_b + da_b, dw_b + w_b, g + dg

    q_1 = Rotation.from_rotvec(dq.reshape(3,))
    q_1 = q * q_1

    return (p_1, v_1, q_1, a_b_1, w_b_1, g_1), Pcov, innovation

