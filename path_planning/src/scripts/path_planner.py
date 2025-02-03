#!/usr/bin/env python3

import rospy
import math
import cv2
import numpy as np
from typing import Union
from std_msgs.msg import Header
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from priority_queue import PriorityQueue
from tf.transformations import quaternion_from_euler


DIRECTIONS_OF_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
DIRECTIONS_OF_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


class PathPlanner:

    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: "tuple[int, int]") -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        return p[1] * mapdata.info.width + p[0]

    @staticmethod
    def get_cell_value(mapdata: OccupancyGrid, p: "tuple[int, int]") -> int:
        """
        Returns the cell corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The cell.
        """
        return mapdata.data[PathPlanner.grid_to_index(mapdata, p)]

    @staticmethod
    def euclidean_distance(
        p1: "tuple[float, float]", p2: "tuple[float, float]"
    ) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: "tuple[int, int]") -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        x = (p[0] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        y = (p[1] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
        return Point(x, y, 0)

    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> "tuple[int, int]":
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        return (x, y)

    @staticmethod
    def path_to_poses(
        mapdata: OccupancyGrid, path: "list[tuple[int, int]]"
    ) -> "list[PoseStamped]":
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        poses = []
        for i in range(len(path) - 1):
            cell = path[i]
            next_cell = path[i + 1]
            if i != len(path) - 1:
                angle_to_next = math.atan2(
                    next_cell[1] - cell[1], next_cell[0] - cell[0]
                )
            q = quaternion_from_euler(0, 0, angle_to_next)
            poses.append(
                PoseStamped(
                    header=Header(frame_id="map"),
                    pose=Pose(
                        position=PathPlanner.grid_to_world(mapdata, cell),
                        orientation=Quaternion(q[0], q[1], q[2], q[3]),
                    ),
                )
            )
        return poses

    @staticmethod
    def is_cell_in_bounds(mapdata: OccupancyGrid, p: "tuple[int, int]") -> bool:
        width = mapdata.info.width
        height = mapdata.info.height
        x = p[0]
        y = p[1]

        if x < 0 or x >= width:
            return False
        if y < 0 or y >= height:
            return False
        return True

    @staticmethod
    def is_cell_walkable(mapdata: OccupancyGrid, p: "tuple[int, int]") -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        if not PathPlanner.is_cell_in_bounds(mapdata, p):
            return False

        WALKABLE_THRESHOLD = 50
        return PathPlanner.get_cell_value(mapdata, p) < WALKABLE_THRESHOLD

    @staticmethod
    def neighbors(
        mapdata: OccupancyGrid,
        p: "tuple[int, int]",
        directions: "list[tuple[int, int]]",
        must_be_walkable: bool = True,
    ) -> "list[tuple[int, int]]":
        """
        Returns the neighbors cells of (x,y) in the occupancy grid given directions to check.
        :param mapdata           [OccupancyGrid] The map information.
        :param p                 [(int, int)]    The coordinate in the grid.
        :param directions        [[(int,int)]]   A list of directions to check for neighbors.
        :param must_be_walkable  [bool]          Whether or not the cells must be walkable
        :return                  [[(int,int)]]   A list of 4-neighbors.
        """
        neighbors = []
        for direction in directions:
            candidate = (p[0] + direction[0], p[1] + direction[1])
            if (
                must_be_walkable and PathPlanner.is_cell_walkable(mapdata, candidate)
            ) or (
                not must_be_walkable
                and PathPlanner.is_cell_in_bounds(mapdata, candidate)
            ):
                neighbors.append(candidate)
        return neighbors

    @staticmethod
    def neighbors_of_4(
        mapdata: OccupancyGrid, p: "tuple[int, int]", must_be_walkable: bool = True
    ) -> "list[tuple[int, int]]":
        return PathPlanner.neighbors(mapdata, p, DIRECTIONS_OF_4, must_be_walkable)

    @staticmethod
    def neighbors_of_8(
        mapdata: OccupancyGrid, p: "tuple[int, int]", must_be_walkable: bool = True
    ) -> "list[tuple[int, int]]":
        return PathPlanner.neighbors(mapdata, p, DIRECTIONS_OF_8, must_be_walkable)

    @staticmethod
    def neighbors_and_distances(
        mapdata: OccupancyGrid,
        p: "tuple[int, int]",
        directions: "list[tuple[int, int]]",
        must_be_walkable: bool = True,
    ) -> "list[tuple[tuple[int, int], float]]":
        """
        Returns the neighbors cells of (x,y) in the occupancy grid given directions to check and their distances.
        :param mapdata           [OccupancyGrid] The map information.
        :param p                 [(int, int)]    The coordinate in the grid.
        :param directions        [[(int,int)]]   A list of directions to check for neighbors.
        :param must_be_walkable  [bool]          Whether or not the cells must be walkable
        :return                  [[(int,int)]]   A list of 4-neighbors.
        """
        neighbors = []
        for direction in directions:
            candidate = (p[0] + direction[0], p[1] + direction[1])
            if not must_be_walkable or PathPlanner.is_cell_walkable(mapdata, candidate):
                distance = PathPlanner.euclidean_distance(direction, (0, 0))
                neighbors.append((candidate, distance))
        return neighbors

    @staticmethod
    def neighbors_and_distances_of_4(
        mapdata: OccupancyGrid, p: "tuple[int, int]", must_be_walkable: bool = True
    ) -> "list[tuple[tuple[int, int], float]]":
        return PathPlanner.neighbors_and_distances(
            mapdata, p, DIRECTIONS_OF_4, must_be_walkable
        )

    @staticmethod
    def neighbors_and_distances_of_8(
        mapdata: OccupancyGrid, p: "tuple[int, int]", must_be_walkable: bool = True
    ) -> "list[tuple[tuple[int, int], float]]":
        return PathPlanner.neighbors_and_distances(
            mapdata, p, DIRECTIONS_OF_8, must_be_walkable
        )

    @staticmethod
    def get_grid_cells(
        mapdata: OccupancyGrid, cells: "list[tuple[int, int]]"
    ) -> GridCells:
        world_cells = []
        for cell in cells:
            world_cells.append(PathPlanner.grid_to_world(mapdata, cell))
        resolution = mapdata.info.resolution
        return GridCells(
            header=Header(frame_id="map"),
            cell_width=resolution,
            cell_height=resolution,
            cells=world_cells,
        )

    @staticmethod
    def calc_cspace(
        mapdata: OccupancyGrid, include_cells: bool
    ) -> "tuple[OccupancyGrid, Union[GridCells, None]]":
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :return        [OccupancyGrid] The C-Space.
        """
        PADDING = 5  # The number of cells around the obstacles

        # Create numpy grid from mapdata
        width = mapdata.info.width
        height = mapdata.info.height
        map = np.array(mapdata.data).reshape(width, height).astype(np.uint8)

        # Get mask of unknown areas
        unknown_area_mask = cv2.inRange(
            map, 255, 255
        )  # -1 overflows to 255 when cast to uint8
        kernel = np.ones((PADDING, PADDING), dtype=np.uint8)
        unknown_area_mask = cv2.erode(unknown_area_mask, kernel, iterations=1)

        # Change unknown areas to free space
        map[map == 255] = 0

        # Inflate the obstacles where necessary
        kernel = np.ones((PADDING, PADDING), np.uint8)
        obstacle_mask = cv2.dilate(map, kernel, iterations=1)
        cspace_data = cv2.bitwise_or(obstacle_mask, unknown_area_mask)
        cspace_data = np.array(cspace_data).reshape(width * height).tolist()

        # Return the C-space
        cspace = OccupancyGrid(
            header=mapdata.header, info=mapdata.info, data=cspace_data
        )

        # Return the cells that were added to the original map
        cspace_cells = None
        if include_cells:
            cells = []

            # Find the indices of the obstacle cells
            obstacle_indices = np.where(obstacle_mask > 0)

            # Convert the indices to cell coordinates and append them to cells
            for y, x in zip(*obstacle_indices):
                cells.append((x, y))

            cspace_cells = PathPlanner.get_grid_cells(mapdata, cells)

        return (cspace, cspace_cells)

    @staticmethod
    def get_cost_map_value(cost_map: np.ndarray, p: "tuple[int, int]") -> int:
        return cost_map[p[1]][p[0]]

    @staticmethod
    def show_map(name: str, map: np.ndarray):
        normalized = cv2.normalize(
            map, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        cv2.imshow(name, normalized)
        cv2.waitKey(0)

    @staticmethod
    def calc_cost_map(mapdata: OccupancyGrid) -> np.ndarray:
        rospy.loginfo("Calculating cost map")

        # Create numpy array from mapdata
        width = mapdata.info.width
        height = mapdata.info.height
        map = np.array(mapdata.data).reshape(height, width).astype(np.uint8)
        map[map == 255] = 100

        # Iteratively dilate the walls until no changes are made
        cost_map = np.zeros_like(map)
        dilated_map = map.copy()
        iterations = 0
        kernel = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], np.uint8)
        while np.any(dilated_map == 0):
            # Increase iterations
            iterations += 1

            # Dilate the map
            next_dilated_map = cv2.dilate(dilated_map, kernel, iterations=1)

            # Get the difference between the next dilated map and the current one to get an outline
            difference = next_dilated_map - dilated_map

            # Assign all non-zero cells in the outline their cost
            difference[difference > 0] = iterations

            # Add the outline to the cost map
            cost_map = cv2.bitwise_or(cost_map, difference)

            # Update dilated map
            dilated_map = next_dilated_map

        # PathPlanner.show_map("wall_dilation", cost_map)

        # Turn the cost map into a mask of only the middle of the hallways
        # All cells that are not in the middle of the hallways will have a cost of 0
        # Cells that are in the middle of the hallways will have a cost of 1
        cost_map = PathPlanner.create_hallway_mask(mapdata, cost_map, iterations // 4)

        # PathPlanner.show_map("hallway_mask", cost_map)

        # Iteratively dilate the hallway mask until no changes are made
        dilated_map = cost_map.copy()
        cost = 1
        for i in range(iterations):
            # Increase cost
            cost += 1

            # Dilate the map
            next_dilated_map = cv2.dilate(dilated_map, kernel, iterations=1)

            # Get the difference between the next dilated map and the current one to get an outline
            difference = next_dilated_map - dilated_map

            # Assign all non-zero cells in the outline their cost
            difference[difference > 0] = cost

            # Add the outline to the cost map
            cost_map = cv2.bitwise_or(cost_map, difference)

            # Update dilated map
            dilated_map = next_dilated_map

        # Subtract 1 from all non-zero values in cost map
        cost_map[cost_map > 0] -= 1

        # PathPlanner.show_map("cost_map", cost_map)

        return cost_map

    @staticmethod
    def create_hallway_mask(
        mapdata: OccupancyGrid, cost_map: np.ndarray, threshold: int
    ) -> np.ndarray:
        """
        Create a mask of the cost_map that only contains cells that are hallway cells.
        """
        # Initialize the mask with the same shape as the cost_map and all values set to False
        mask = np.zeros_like(cost_map, dtype=bool)

        # Get the indices of the non-zero cells in the cost_map
        non_zero_indices = np.nonzero(cost_map)

        # Iterate over the non-zero cells in the cost_map
        for y, x in zip(*non_zero_indices):
            # If the cell is a hallway cell, set the corresponding cell in the mask to True
            if PathPlanner.is_hallway_cell(mapdata, cost_map, (x, y), threshold):
                mask[y][x] = 1

        return mask.astype(np.uint8)

    @staticmethod
    def is_hallway_cell(
        mapdata: OccupancyGrid,
        cost_map: np.ndarray,
        p: "tuple[int, int]",
        threshold: int,
    ) -> bool:
        """
        Determine whether a cell is a "hallway cell" meaning it has a cost
        greater than or equal to all of its neighbors
        """
        cost_map_value = PathPlanner.get_cost_map_value(cost_map, p)
        for neighbor in PathPlanner.neighbors_of_8(mapdata, p, False):
            neighbor_cost_map_value = PathPlanner.get_cost_map_value(cost_map, neighbor)
            if (
                neighbor_cost_map_value < threshold
                or neighbor_cost_map_value > cost_map_value
            ):
                return False
        return True

    @staticmethod
    def get_first_walkable_neighbor(
        mapdata, start: "tuple[int, int]"
    ) -> "tuple[int, int]":
        """
        Helper function for a_star that gets the first walkable neighbor from
        the start cell in case it is not already walkable
        :param mapdata [OccupancyGrid] The map data.
        :param padding [start]         The start cell.
        :return        [(int, int)]    The first walkable neighbor.
        """
        # Create queue for breadth-first search
        queue = []
        queue.append(start)

        # Initialize dictionary for keeping track of visited cells
        visited = {}

        while queue:
            current = queue.pop(0)
            if PathPlanner.is_cell_walkable(mapdata, current):
                return current

            for neighbor in PathPlanner.neighbors_of_4(mapdata, current, False):
                visited[neighbor] = True
                queue.append(neighbor)

        # If nothing found, just return original start cell
        return start

    @staticmethod
    def a_star(
        mapdata: OccupancyGrid,
        cost_map: np.ndarray,
        start: "tuple[int, int]",
        goal: "tuple[int, int]",
    ) -> "tuple[Union[list[tuple[int, int]], None], Union[float, None], tuple[int, int], tuple[int, int]]":
        COST_MAP_WEIGHT = 1000

        # If the start cell is not walkable, get the first walkable neighbor instead
        if not PathPlanner.is_cell_walkable(mapdata, start):
            start = PathPlanner.get_first_walkable_neighbor(mapdata, start)

        # Likewise, if the goal cell is not walkable, get the first walkable neighbor instead
        if not PathPlanner.is_cell_walkable(mapdata, goal):
            goal = PathPlanner.get_first_walkable_neighbor(mapdata, goal)

        pq = PriorityQueue()
        pq.put(start, 0)

        cost_so_far = {}
        distance_cost_so_far = {}
        cost_so_far[start] = 0
        distance_cost_so_far[start] = 0
        came_from = {}
        came_from[start] = None

        while not pq.empty():
            current = pq.get()

            if current == goal:
                break

            for neighbor, distance in PathPlanner.neighbors_and_distances_of_8(
                mapdata, current
            ):
                added_cost = (
                    distance
                    + COST_MAP_WEIGHT
                    * PathPlanner.get_cost_map_value(cost_map, neighbor)
                )
                new_cost = cost_so_far[current] + added_cost
                if not neighbor in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    distance_cost_so_far[neighbor] = (
                        distance_cost_so_far[current] + distance
                    )
                    priority = new_cost + PathPlanner.euclidean_distance(neighbor, goal)
                    pq.put(neighbor, priority)
                    came_from[neighbor] = current

        path = []
        cell = goal

        while cell:
            path.insert(0, cell)

            if cell in came_from:
                cell = came_from[cell]
            else:
                return (None, None, start, goal)

        # Prevent paths that are too short
        MIN_PATH_LENGTH = 12
        if len(path) < MIN_PATH_LENGTH:
            return (None, None, start, goal)

        # Truncate the last few poses of the path
        POSES_TO_TRUNCATE = 8
        path = path[:-POSES_TO_TRUNCATE]

        return (path, distance_cost_so_far[goal], start, goal)

    @staticmethod
    def path_to_message(mapdata: OccupancyGrid, path: "list[tuple[int, int]]") -> Path:
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        poses = PathPlanner.path_to_poses(mapdata, path)
        return Path(header=Header(frame_id="map"), poses=poses)