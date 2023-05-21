from flightsim.world import World
import numpy as np
from collections import defaultdict
import math

from ..occupancy_map import OccupancyMap

class PathPlanningAlgorithms( object ):

    relative_index_of_node_neighbor = np.stack( np.meshgrid( [-1, 0, 1], [-1, 0, 1], [-1, 0, 1] ), axis=-1).reshape(-1, 3)
    relative_index_of_node_neighbor = np.delete( relative_index_of_node_neighbor, 13, axis=0 )

    def __init__( self, start_index: tuple, goal_index: tuple, start_pos: tuple, goal_pos: tuple, astar=False ) -> None:
        """
        Initialize the PathPlanningAlgorithms class
        """

        self.start = start_index
        self.goal = goal_index
        self.start_point_in_world = start_pos
        self.goal_in_world = goal_pos

        self.explored_set_of_grid_points = set()
        self.priority_queue_containing_nodes = list()

        # creates a dictionary to store distances and initialises each distance to infinity
        self.heuristic_cost_from_node = defaultdict( lambda: float( 'inf' ) )
        self.parent_node_of_current_node = dict()

        # priority queue uses the first element of tuple to heapify. In our case, this is the distance from start node.
        self.priority_queue_containing_nodes.append( ( 0, start_index ) )

        # setting heuristic of the start node as 0.
        self.heuristic_cost_from_node[ start_index ] = 0

    def explore_neighbors( self, node: np.array, occ_map: OccupancyMap ):
        """
        Explore the neighbors of the current node
        Parameters:
            node,   Current node of the quadrotor
            occ_map,    OccupancyMap object representing the environment obstacles
        Output:
            neighbors_free_from_obstacles,   Neighbors of the current node that are not obstacles
        """
        neighbor_offset_from_current_node =  node + PathPlanningAlgorithms.relative_index_of_node_neighbor
        # Creating a mask to find the neighbors that are within the world
        mask_for_neighbors_within_bounds_1 = (neighbor_offset_from_current_node >=0 )
        mask_for_neighbors_within_bounds_2 = (neighbor_offset_from_current_node < occ_map.map.shape )
        mask_for_neighbors_within_bounds = ( mask_for_neighbors_within_bounds_1 & mask_for_neighbors_within_bounds_2 )
        mask_for_neighbors_within_bounds = mask_for_neighbors_within_bounds.all( axis = 1 )
        neighbor_offset_from_current_node = neighbor_offset_from_current_node[ mask_for_neighbors_within_bounds, :]
        # Finding neighbors free from obstacles
        neighbors_free_from_obstacles = neighbor_offset_from_current_node[ np.where( occ_map.map[ neighbor_offset_from_current_node[:, 0], neighbor_offset_from_current_node[:, 1], neighbor_offset_from_current_node[:, 2 ] ] == 0 ) ]
        return neighbors_free_from_obstacles

    def find_norm( self, node1: list, node2: list ):
        """
        Find the L2 norm of the two nodes using the formula instead of numpy since numpy library is slow
        Parameters:
            node1,  First node
            node2,  Second node
        Output:
            norm,   L2 norm of the two nodes
        """
        return math.sqrt( (node1[0] - node2[0])**2 + (node1[1] - node2[1])**2 + (node1[2] - node2[2])**2 )

    def reached_the_end_goal( self, occ_map: OccupancyMap ):
        """
        When quadrotor reaches goal, returning the optimal path traced by it.
        Parameters:
            occ_map,    OccupancyMap object representing the environment obstacles
        Output:
            quadrotor_trajectory,   Optimal path traced by the quadrotor
            len( self.explored_set_of_grid_points ),   Number of nodes explored by the quadrotor
        """
        quadrotor_trajectory = list()
        quadrotor_trajectory.append( self.goal_in_world )
        current_node = self.goal

        while current_node is not None:
            parent_of_current_node = self.parent_node_of_current_node[ current_node ]
            if parent_of_current_node == self.start:
                quadrotor_trajectory.insert(0, self.start_point_in_world )
                break

            quadrotor_trajectory.insert(0, occ_map.index_to_metric_center( parent_of_current_node ) )
            current_node = parent_of_current_node
        
        return np.array( quadrotor_trajectory), len( self.explored_set_of_grid_points )

    def graph_search( self ):
        """
        Graph search algorithm
        """
        pass