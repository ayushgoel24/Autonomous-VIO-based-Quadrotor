import numpy as np
from scipy.sparse.linalg import spsolve
from scipy.sparse import lil_matrix

from .graph_search import graph_search
from .environment import ApplicationProperties
from .helper import Helper

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

        # resolution and margin parameters to use for path planning
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.5

        # dense path returned from your Dijkstra or AStar graph search algorithm
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        self.applicationProperties = ApplicationProperties("proj3/code/application.yml")
        self.applicationProperties.initializeProperties()

        self.velocity = self.applicationProperties.get_property_value( "quadrotor.velocity" )

        # generate a sparse set of waypoints to fly between
        self.sparse_waypoints = self.find_sparse_trajectory_waypoints( self.path )
        self.number_of_waypoints = self.sparse_waypoints.shape[0]
        print(f"self.number_of_waypoints: {self.number_of_waypoints}")
        
        self.distance_of_each_segment = np.linalg.norm( self.sparse_waypoints[1:] - self.sparse_waypoints[:-1], axis=1 )[:, np.newaxis]
        duration_of_each_segment = self.distance_of_each_segment / self.velocity
        duration_of_each_segment = Helper.rectify_time( self.applicationProperties, duration_of_each_segment )

        self.start_time_of_each_segment = np.zeros(( self.number_of_waypoints, 1 ))
        self.start_time_of_each_segment[1:] = np.cumsum( duration_of_each_segment )[:, np.newaxis]
        self.start_time_of_each_segment = self.start_time_of_each_segment.flatten()

        self.total_params_for_journey = self.applicationProperties.get_property_value( "path.num_pts" ) * ( self.number_of_waypoints - 1 )

        self.duration_of_each_segment = Helper.restrict_time( self.applicationProperties, duration_of_each_segment )
        self.total_duration = np.sum( self.duration_of_each_segment )

        self.points = self.sparse_waypoints


    def find_sparse_trajectory_waypoints( self, waypoints ):
        """
        Given a set of points and a threshold, returns a subset of sparse points 
        where no two points are closer than the threshold.
        """
        if waypoints.shape[0] < self.applicationProperties.get_property_value( "path.min_path_threshold" ):
            return waypoints

        index_of_path_change = 0
        max_change_in_direction = 0
        
        for k in range(1, len( waypoints ) - 1):
            change_in_direction = np.linalg.norm( np.cross( ( waypoints[0] - waypoints[k] ), ( waypoints[-1] - waypoints[0] ) ) ) / np.linalg.norm( waypoints[-1] - waypoints[0] )
            if change_in_direction > max_change_in_direction:
                index_of_path_change = k
                max_change_in_direction = change_in_direction

        if max_change_in_direction > self.applicationProperties.get_property_value( "path.threshold" ):
            return np.vstack(( self.find_sparse_trajectory_waypoints( waypoints[ : index_of_path_change + 1 ] )[:-1], self.find_sparse_trajectory_waypoints( waypoints[ index_of_path_change: ] ) ))
        else:
            if np.linalg.norm( waypoints[-1] - waypoints[0] ) > 2:
                return np.vstack(( waypoints[0],  waypoints[ len(waypoints) // 2 ], waypoints[-1]))
            else:
                return np.vstack(( waypoints[0], waypoints[-1] ))
            

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

        A_matrix = lil_matrix(( self.total_params_for_journey, self.total_params_for_journey ))
        B_matrix = Helper.construct_B_matrix( self.applicationProperties.get_property_value( "path.num_pts" ), self.sparse_waypoints )
        
        segment_trajectory = 0
        for time_segment in self.duration_of_each_segment:
            if segment_trajectory != len( self.duration_of_each_segment ) - 1:
                A_matrix = Helper.construct_A_matrix( A_matrix, self.applicationProperties.get_property_value( "path.num_pts" ), time_segment, segment_trajectory, False )
            else:
                A_matrix = Helper.construct_A_matrix( A_matrix, self.applicationProperties.get_property_value( "path.num_pts" ), time_segment, segment_trajectory, True )
            segment_trajectory += 1

        X_matrix = spsolve( A_matrix.tocsc(), B_matrix ).toarray()

        if t > self.total_duration:
            x = self.sparse_waypoints[-1]
        
        else:
            current_segment = np.where( self.start_time_of_each_segment <= t )[0][-1]

            time_periods = Helper.compute_time( t, self.start_time_of_each_segment[current_segment] )

            x = Helper.compute_position( time_periods, X_matrix, self.applicationProperties.get_property_value( "path.num_pts" ), current_segment )
            x_dot = Helper.compute_velocity( time_periods, X_matrix, self.applicationProperties.get_property_value( "path.num_pts" ), current_segment )
            x_ddot = Helper.compute_acceleration( time_periods, X_matrix, self.applicationProperties.get_property_value( "path.num_pts" ), current_segment )
            x_dddot = Helper.compute_jerk( time_periods, X_matrix, self.applicationProperties.get_property_value( "path.num_pts" ), current_segment )
            x_ddddot = Helper.compute_snap( time_periods, X_matrix, self.applicationProperties.get_property_value( "path.num_pts" ), current_segment )

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
