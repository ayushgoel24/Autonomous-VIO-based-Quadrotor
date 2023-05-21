import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control( object ):
    """

    """
    def __init__( self, quad_params ):
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

        # defining thrust_drag_matrix
        gamma = self.k_drag / self.k_thrust
        thrust_drag_matrix = np.array([
            [ 1, 1, 1, 1 ],
            [ 0, self.arm_length, 0, -self.arm_length ],
            [ -self.arm_length, 0, self.arm_length, 0 ],
            [ gamma, -gamma, gamma, -gamma ]
        ])

        self.thrust_drag_matrix_inverse = np.linalg.inv( thrust_drag_matrix )

        # STUDENT CODE HERE

        # defining PD gains
        # TODO: adjust the gains here
        self.proportional_gain  = np.diag( np.array([ 8, 8, 19 ], dtype=float) )
        self.derivative_gain    = np.diag( np.array([ 5.5, 5.5, 8.7 ], dtype=float) )
        self.rotation_gain      = np.diag( np.array([ 2812, 2812, 163 ], dtype=float) )
        self.angular_gain       = np.diag( np.array([ 128, 128, 73 ], dtype=float) )

    def update( self, t, state, flat_output ):
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
        cmd_thrust       = 0
        cmd_moment       = np.zeros((3,))
        cmd_q            = np.zeros((4,))

        # STUDENT CODE HERE

        # desired quad trajectories
        desired_x        = flat_output['x'].reshape(3, 1)
        desired_x_dot    = flat_output['x_dot'].reshape(3, 1)
        desired_x_ddot   = flat_output['x_ddot'].reshape(3, 1)
        desired_x_dddot  = flat_output['x_dddot'].reshape(3, 1)
        desired_x_ddddot = flat_output['x_ddddot'].reshape(3, 1)
        desired_yaw      = flat_output['yaw']
        desired_yaw_dot  = flat_output['yaw_dot']

        # current quad states
        current_x        = state['x'].reshape(3, 1)
        current_v        = state['v'].reshape(3, 1)
        current_q        = state['q'].reshape(4,)
        current_w        = state['w'].reshape(3, 1)

        error_in_position = current_x - desired_x
        error_in_velocity = current_v - desired_x_dot

        commanded_acceleration = desired_x_ddot - self.derivative_gain @ error_in_velocity - self.proportional_gain @ error_in_position
        total_commanded_force = ( self.mass * commanded_acceleration ) + np.array([ [0], [0], [self.mass * self.g] ])

        # Converting quaternion to rotation matrix
        # Reference: (https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html)
        current_R = Rotation.from_quat(current_q).as_matrix()

        # computing b3 using eq. 33
        b_3 = current_R @ np.array([ [0], [0], [1] ])

        # computing u1 using eq 34
        u_1 = b_3.T @ total_commanded_force
        
        # computing desired b3 using eq 35
        desired_b_3 = total_commanded_force / np.linalg.norm( total_commanded_force )

        # vector defining yaw dirn in (a1, a2) plane (ref eq 36)
        a_psi = np.array([ [ np.cos( desired_yaw ) ], [ np.sin( desired_yaw ) ], [0] ])

        # computing desired b2 axis perpendicular to both b3 desired and a_psi (ref eq 37)
        desired_b_2 = ( np.cross( desired_b_3.T, a_psi.T ) / np.linalg.norm( np.cross( desired_b_3.T, a_psi.T ) ) ).T

        # Computing R_desired by eq 38
        desired_R = np.hstack( ( np.cross( desired_b_2.T, desired_b_3.T ).T, desired_b_2, desired_b_3 ) )

        error_in_rotation_internal = desired_R.T @ current_R - current_R.T @ desired_R
        error_in_rotation = 0.5 * np.array([ [ error_in_rotation_internal[2, 1] ], [ error_in_rotation_internal[0, 2] ], [ error_in_rotation_internal[1, 0] ] ])

        # computing u2
        u_2 = self.inertia @ ( - self.rotation_gain @ error_in_rotation - self.angular_gain @ current_w )

        u = np.vstack(( u_1, u_2 ))

        # computing motor speeds
        motor_speeds = self.thrust_drag_matrix_inverse @ u
        cmd_motor_speeds = np.sign( motor_speeds ) * np.sqrt( np.absolute( motor_speeds ) / self.k_thrust )
        cmd_motor_speeds = np.clip( cmd_motor_speeds, self.rotor_speed_min, self.rotor_speed_max )
        
        cmd_thrust = u_1.flatten()
        cmd_moment = u_2
        cmd_q = Rotation.from_matrix(desired_R).as_quat()

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}

        return control_input
