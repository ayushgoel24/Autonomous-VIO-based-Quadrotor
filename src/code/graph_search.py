from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.
from .planning import AStar, Dijkstra

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """
    
    # Create an occupancy map instance from the world, resolution and margin.
    occ_map = OccupancyMap( world, resolution, margin )
    
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple( occ_map.metric_to_index( start ) )
    goal_index = tuple( occ_map.metric_to_index( goal ) )

    # Create an instance of the search algorithm.
    path_planning_object = AStar( start_index, goal_index, start, goal ) if astar else Dijkstra( start_index, goal_index, start, goal )
    return path_planning_object.graph_search( occ_map )