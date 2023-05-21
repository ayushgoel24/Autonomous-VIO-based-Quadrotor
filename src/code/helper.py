import numpy as np
from scipy.sparse import lil_matrix
import math

class Helper( object ):

    @staticmethod
    def construct_time_derivatives( time ):
        
        time_derivatives = np.array([
            [0, 0, 0, 0, 0, 0, 0, 1 ],
            [ time**7, time**6, time**5, time**4, time**3, time**2, time, 1 ],
            [ 7 * time**6, math.factorial( 3 ) * time**5, 5 * time**4, 2 * math.factorial( 2 ) * time**3, 3 * time**2, math.factorial( 2 ) * time, 1, 0 ],
            [ 7 * math.factorial( 3 ) * time**5, 5 * math.factorial( 3 ) * time**4, 10 * math.factorial( 2 ) * time**3, 2 * math.factorial( 3 ) * time**2, math.factorial( 3 ) * time, 2, 0, 0 ],
            [ 1.75 * math.factorial( 5 ) * time**4, math.factorial( 5 ) * time**3, 10 * math.factorial( 3 ) * time**2, math.factorial( 4 ) * time, math.factorial( 3 ), 0, 0, 0 ],
            [ 7 * math.factorial( 5 ) * time**3, 3 * math.factorial( 5 ) * time**2, math.factorial( 5 ) * time, math.factorial( 4 ), 0, 0, 0, 0 ],
            [ 21 * math.factorial( 5 ) * time**2, math.factorial( 6 ) * time, math.factorial( 5 ), 0, 0, 0, 0, 0 ],
            [ math.factorial( 7 ) * time, math.factorial( 6 ), 0, 0, 0, 0, 0, 0 ]
        ])
        return time_derivatives

    
    @staticmethod
    # @profile
    def construct_B_matrix( mins_num_pts, sparse_waypoints ):
        B_matrix = lil_matrix(( mins_num_pts * ( sparse_waypoints.shape[0] - 1 ), 3 ))
        for i in range( sparse_waypoints.shape[0] - 1 ):
            B_matrix[ mins_num_pts * i + 3 ] = sparse_waypoints[i]
            B_matrix[ mins_num_pts * i + 4 ] = sparse_waypoints[i + 1]
        return B_matrix
    

    @staticmethod
    def construct_A_matrix( A_matrix, mins_num_pts, time_segment, segment_trajectory, last_iteration=False ):
        A_matrix[ 0, 6 ] = 1
        A_matrix[ 1, 5 ] = 2
        A_matrix[ 2, 4 ] = math.factorial( 3 )

        if last_iteration:
            A_matrix[ mins_num_pts*segment_trajectory + 3 : mins_num_pts*segment_trajectory + 8, mins_num_pts*segment_trajectory:mins_num_pts*segment_trajectory + 8] = Helper.construct_time_derivatives( time_segment )[:5, :]
        else:
            A_matrix[ [ mins_num_pts*segment_trajectory + 5, mins_num_pts*segment_trajectory + 10, mins_num_pts*segment_trajectory + 6, mins_num_pts*segment_trajectory + 8, mins_num_pts*segment_trajectory + 9, mins_num_pts*segment_trajectory + 7 ], [mins_num_pts*segment_trajectory + 14, mins_num_pts*segment_trajectory + 9, mins_num_pts*segment_trajectory + 13, mins_num_pts*segment_trajectory + 11, mins_num_pts*segment_trajectory + 10, mins_num_pts*segment_trajectory + 12 ] ] = [ -1, -1 * math.factorial( 6 ), -1 * math.factorial( 2 ), -1 * math.factorial( 4 ), -1 * math.factorial( 5 ), -1 * math.factorial( 3 ) ]
            A_matrix[ mins_num_pts*segment_trajectory + 3 : mins_num_pts*segment_trajectory + 11, mins_num_pts*segment_trajectory : mins_num_pts*segment_trajectory + 8] = Helper.construct_time_derivatives( time_segment )

        return A_matrix


    @staticmethod
    def rectify_time( applicationProperties, duration_of_each_segment ):
        for k in ( -1, 0 ):
            duration_of_each_segment[ k ] *= applicationProperties.get_property_value( "time.factor" )
        duration_of_each_segment *= np.sqrt( applicationProperties.get_property_value( "time.coeff" ) / duration_of_each_segment )
        return duration_of_each_segment


    @staticmethod
    def compute_time( current_time, start_time ):
        time_difference = current_time - start_time
        time_periods = np.array([
            [ time_difference**7, time_difference**6, time_difference**5, time_difference**4, time_difference**3, time_difference**2, time_difference, 1 ],
            [ 7 * time_difference**6, 6 * time_difference**5, 5 * time_difference**4, 4 * time_difference**3, 3 * time_difference**2, 2 * time_difference, 1, 0 ],
            [ 42 * time_difference**5, 30 * time_difference**4, 20 * time_difference**3, 12 * time_difference**2, 6 * time_difference, 2, 0, 0 ],
            [ 210 * time_difference**4, 120 * time_difference**3, 60 * time_difference**2, 24 * time_difference, 6, 0, 0, 0 ],
            [ 840 * time_difference**3, 360 * time_difference**2, 120 * time_difference, 24, 0, 0, 0, 0 ]
        ], dtype=float)
        return time_periods

    
    @staticmethod
    def compute_position( time_periods, X_matrix, mins_num_pts, current_segment ):
        return ( time_periods @ X_matrix[ mins_num_pts*current_segment : mins_num_pts*current_segment + 8, : ] )[0, :]

    
    @staticmethod
    def compute_velocity( time_periods, X_matrix, mins_num_pts, current_segment ):
        return ( time_periods @ X_matrix[ mins_num_pts*current_segment : mins_num_pts*current_segment + 8, : ] )[1, :]

    
    @staticmethod
    def compute_acceleration( time_periods, X_matrix, mins_num_pts, current_segment ):
        return ( time_periods @ X_matrix[ mins_num_pts*current_segment : mins_num_pts*current_segment + 8, : ] )[2, :]

    
    @staticmethod
    def compute_jerk( time_periods, X_matrix, mins_num_pts, current_segment ):
        return ( time_periods @ X_matrix[ mins_num_pts*current_segment : mins_num_pts*current_segment + 8, : ] )[3, :]


    @staticmethod
    def compute_snap( time_periods, X_matrix, mins_num_pts, current_segment ):
        return ( time_periods @ X_matrix[ mins_num_pts*current_segment : mins_num_pts*current_segment + 8, : ] )[4, :]

    @staticmethod
    def restrict_time( applicationProperties, duration_of_each_segment ):
        return duration_of_each_segment.clip( applicationProperties.get_property_value( "time.clip_lower" ), np.inf ).flatten()