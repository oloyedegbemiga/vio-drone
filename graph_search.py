from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

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

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    occ_grid = occ_map.map # numpy array of T/F

    moves = [(0, 1), (0, -1), (-1, 0), (1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)] # 2D
    moves_3d = [(0, 0, 1), (0, 0, -1)] # start position has nbrs above and below 
    for move in moves:
        x, y = move
        move_3d_pos = (x, y, 1) # moves above plane
        move_3d_neg = (x, y, -1) # moves below plane
        move_3d_zero = (x, y, 0) # moves on the same plane
        moves_3d.extend([move_3d_pos, move_3d_neg, move_3d_zero])

    x,y,z = 0, 1, 2
    boundaries = occ_grid.shape


    if occ_map.is_occupied_index(goal_index):
        print("goal occ")
        return None
    
    if occ_map.is_occupied_index(start_index):
        print("goal start")
        return None

    if occ_map.is_occupied_metric(start) or occ_map.is_occupied_metric(goal):
        print("goal or start occ")
        return None

    # start_x = occ_grid.metric_to_grid(start[])

    def convert_path_np(path_):
        metric_path = np.empty((len(path_) + 2, 3))
        P = len(path_)
        N,d = metric_path.shape

        metric_path[0] = start
        metric_path[N - 1] = goal
        cursor = 1
        for i in range(0, P):
            idx_point = np.array(path_[i])
            metric_point = occ_map.index_to_metric_center(idx_point)
            metric_path[cursor] = metric_point
            cursor += 1

        if np.array_equal(metric_path[0], metric_path[1]):
            metric_path = metric_path[1:]

        if np.array_equal(metric_path[N - 2], metric_path[N - 1]):
            metric_path = metric_path[:N - 2]
        


        return metric_path

    # def compute_heurstics(goal_index)

    def compute_cost(move):
        return np.sqrt(move[x]**2 + move[y]**2 + move[z]**2)
        # return np.linalg.norm(np.array(move))
        # return 1

    def heuristic(heurist_, curr_voxel, goal_voxel):
        return (goal_voxel[x] - curr_voxel[x]) ** 2 + (goal_voxel[y] - curr_voxel[y]) ** 2 + (goal_voxel[z] - curr_voxel[z])**2
    
    
    def is_valid_node(nbr_x, nbr_y, nbr_z):
        test_voxel = np.array([nbr_x, nbr_y, nbr_z])
        return not occ_map.is_occupied_index(test_voxel) and occ_map.is_valid_index(test_voxel)


    def get_path(sources_, goal_idx):
        path = []
        voxel = goal_idx
        # print("entersx")
        while voxel in sources_:
            # print(voxel)
            vx = occ_map.index_to_metric_center(voxel)
            path.append(vx)
            voxel = sources_[voxel]
        # print("exits")
        path.append(occ_map.index_to_metric_center(voxel))
        return np.array(path[::-1])
        # metric_path = np.empty((len(path_) + 2, 3))


    p_q = [(0, start_index)]
    # p_q = [(0, start_index, [])]
    visited = set()
    distances = {(start_index[x], start_index[y], start_index[z]): 0}
    expanded_ = 0
    source_nodes = {}

    # assert p_q == ""
    while p_q:
        cost, voxel = heappop(p_q)

        if (voxel[x], voxel[y], voxel[x]) in visited:
            continue


        visited.add((voxel[-x], voxel[x], voxel[z]))
        
        if voxel == goal_index:
            path_ = get_path(source_nodes,goal_index)
            return (path_, expanded_ + 2)

        # assert is_valid_node(goal_index[x], goal_index[y], goal_index[z])
        for move in moves_3d:
            nbr_x, nbr_y, nbr_z = voxel[x] + move[x], voxel[y] + move[y], voxel[z] + move[z]
            nbr_tuple = (nbr_x, nbr_y, nbr_z)

            nbr_metric = occ_map.index_to_metric_center(nbr_tuple)
            voxel_metric = occ_map.index_to_metric_center(voxel)
            if is_valid_node(nbr_x, nbr_y, nbr_z):
                heuristic_type = "0" if astar else "Euclidean"

                heuristic_ = heuristic(heuristic_type, nbr_tuple, goal_index)
                move_diff = nbr_metric + voxel_metric
                cost_tmp = cost + compute_cost(move_diff)
                cost_prime = heuristic_ + cost_tmp

                if nbr_tuple not in distances or cost_prime < distances[nbr_tuple]:
                    distances[nbr_tuple] = cost_prime
                    source_nodes[(nbr_x, nbr_y, nbr_z)] = (voxel[x], voxel[y], voxel[z])
                    heappush(p_q, (cost_prime, nbr_tuple))
                    expanded_ += 1


    return None
