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
    new_p = np.zeros((3, 1))
    new_v = np.zeros((3, 1))
    new_q = Rotation.identity()

    rotation_matrix = Rotation.as_matrix( q )

    new_p = p + ( v * dt ) + ( 0.5 ) * ( rotation_matrix @ ( a_m - a_b ) + g ) * ( dt * dt )
    new_v = v + ( rotation_matrix @ ( a_m - a_b ) + g ) * dt
    new_q = q * Rotation.from_rotvec( ( ( w_m - w_b ) * dt ).reshape(( 3, )) )

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
    F_x = np.identity( 18 )
    F_x[ 0:3, 3:6 ] = np.identity( 3 ) * dt

    rotation_matrix = Rotation.as_matrix( q )
    acc_diff = a_m - a_b

    F_x[ 3:6, 6:9 ] = -rotation_matrix @ np.array([
        [ 0, -acc_diff[2][0], acc_diff[1][0] ], 
        [ acc_diff[2][0], 0, -acc_diff[0][0] ], 
        [ -acc_diff[1][0], acc_diff[0][0], 0 ]
    ]) * dt
    F_x[ 3:6, 9:12 ] = -rotation_matrix * dt
    F_x[ 3:6, 15:18 ] = np.identity( 3 ) * dt
    F_x[ 6:9, 6:9 ] = np.transpose( Rotation.as_matrix( Rotation.from_rotvec( ((w_m - w_b) * dt).reshape(3, )) ) )
    F_x[ 6:9, 12:15 ] = -np.identity( 3 ) * dt

    Q_i = np.zeros((12, 12))
    Q_i[0:3, 0:3] = ( accelerometer_noise_density ** 2 ) * ( dt ** 2 ) * np.identity( 3 )
    Q_i[3:6, 3:6] = ( gyroscope_noise_density ** 2 ) * ( dt ** 2 ) * np.identity( 3 )
    Q_i[6:9, 6:9] = ( accelerometer_random_walk ** 2 ) * dt * np.identity( 3 )
    Q_i[9:12, 9:12] = ( gyroscope_random_walk ** 2 ) * dt * np.identity( 3 )

    F_i = np.zeros(( 18, 12 ))
    F_i[ 3:6, 0:3 ] = np.identity( 3 )
    F_i[ 6:9, 3:6 ] = np.identity( 3 )
    F_i[ 9:12, 6:9 ] = np.identity( 3 )
    F_i[ 12:15, 9:12 ] = np.identity( 3 )

    return ( F_x @ error_state_covariance @ F_x.T ) + ( F_i @ Q_i @ F_i.T )


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

    rotation_matrix = Rotation.as_matrix( q )

    P_c = ( rotation_matrix.T @ ( Pw - p ) ).reshape( 3, )
    normalized_P_c = P_c / P_c[2]

    # YOUR CODE HERE - compute the innovation next state, next error_state covariance
    innovation = uv - np.array([ normalized_P_c[0], normalized_P_c[1] ]).reshape(2, 1)

    if np.linalg.norm( innovation ) < error_threshold:

        dzt_dPc = ( 1 / P_c[2] ) * np.array([
            [ 1, 0, -normalized_P_c[0] ],
            [ 0, 1, -normalized_P_c[1] ]
        ])

        dPc_ddt = np.array([
            [ 0, -P_c[2], P_c[1] ],
            [ P_c[2], 0, -P_c[0] ],
            [ -P_c[1], P_c[0], 0 ]
        ])

        dPc_ddp = -rotation_matrix.T

        dzt_ddt = dzt_dPc @ dPc_ddt
        dzt_ddp = dzt_dPc @ dPc_ddp

        H = np.zeros(( 2, 18 ))
        H[ 0:2, 0:3 ] = dzt_ddp
        H[ 0:2, 6:9 ] = dzt_ddt

        K_t = error_state_covariance @ H.T @ np.linalg.inv( ( H @ error_state_covariance @ H.T ) + Q )
        d_x = ( K_t @ innovation ).reshape( 18, 1 )

        p += d_x[0:3]
        v += d_x[3:6]
        q = q * Rotation.from_rotvec( ( d_x[6:9] ).reshape(3, ) )
        a_b += d_x[9:12]
        w_b += d_x[12:15]
        g += d_x[15:18]

        error_state_covariance = ( np.identity( 18 ) - K_t @ H ) @ error_state_covariance @ ( np.identity( 18 ) - K_t @ H ).T + K_t @ Q @ K_t.T

    return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
