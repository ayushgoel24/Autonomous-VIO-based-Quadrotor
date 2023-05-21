from heapq import heappush, heappop
from ..occupancy_map import OccupancyMap
import numpy as np
from .planning_algorithms import PathPlanningAlgorithms

class AStar( PathPlanningAlgorithms ):

    def __init__( self, start_index: tuple, goal_index: tuple, start_pos: tuple, goal_pos: tuple ) -> None:
        """
        Initialize the AStar class
        """
        super().__init__( start_index, goal_index, start_pos, goal_pos )

    def graph_search( self, occ_map: OccupancyMap ):
        """
        Code to implement A* algorithm

        Parameters:
            occ_map,    OccupancyMap object representing the environment obstacles
        Output:
            return a tuple (path, nodes_expanded)
        """
        while( self.priority_queue_containing_nodes ):
            # index of the current node
            current_node_of_quad = heappop( self.priority_queue_containing_nodes )[1]
            current_node_of_quad_array = np.array( current_node_of_quad )
            
            # end on reaching goal
            if current_node_of_quad[0] == self.goal[0] and current_node_of_quad[1] == self.goal[1] and current_node_of_quad[2] == self.goal[2]:
                return self.reached_the_end_goal( occ_map )
            
            # marking node as visited
            self.explored_set_of_grid_points.add( current_node_of_quad )
            
            # finds all neighbors of current node
            neighbors_of_current_node = self.explore_neighbors( current_node_of_quad_array, occ_map )
            for neighbor_of_current_node in neighbors_of_current_node:
                heuristic_distance = self.heuristic_cost_from_node[ current_node_of_quad ] + self.find_norm( neighbor_of_current_node, current_node_of_quad_array )
                total_distance = heuristic_distance + self.find_norm( neighbor_of_current_node, self.goal )
                if heuristic_distance < self.heuristic_cost_from_node[ tuple( neighbor_of_current_node.tolist() ) ]:
                    neighbor_of_current_node_tuple = tuple( neighbor_of_current_node.tolist() )
                    self.heuristic_cost_from_node[ neighbor_of_current_node_tuple ] = heuristic_distance
                    self.parent_node_of_current_node[ neighbor_of_current_node_tuple ] = current_node_of_quad
                    heappush( self.priority_queue_containing_nodes, ( total_distance, neighbor_of_current_node_tuple ) )

        return None, 0
