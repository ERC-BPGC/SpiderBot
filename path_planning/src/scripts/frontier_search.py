#!/usr/bin/env python3

from path_planner import PathPlanner
from nav_msgs.msg import OccupancyGrid
from path_planning.msg import Frontier, FrontierList


class FrontierSearch:
    @staticmethod
    def search(
        mapdata: OccupancyGrid,
        start: "tuple[int, int]",
        include_frontier_cells: bool = False,
    ) -> "tuple[FrontierList, list[tuple[int, int]]]":
        MIN_FRONTIER_SIZE = 8  # Number of cells

        # Create queue for breadth-first search
        queue = []
        queue.append(start)

        # Initialize dictionaries for keeping track of visited and frontier cells
        visited = {}
        is_frontier = {}
        visited[start] = True

        # Initialize list of frontiers
        frontiers = []

        # Initialize list of frontier cells
        frontier_cells = []

        while queue:
            current = queue.pop(0)
            for neighbor in PathPlanner.neighbors_of_4(mapdata, current):
                neighbor_value = PathPlanner.get_cell_value(mapdata, neighbor)
                if neighbor_value >= 0 and not neighbor in visited:
                    visited[neighbor] = True
                    queue.append(neighbor)
                elif FrontierSearch.is_new_frontier_cell(
                    mapdata, neighbor, is_frontier
                ):
                    # Mark as frontier
                    is_frontier[neighbor] = True

                    # Build new frontier
                    new_frontier, new_frontier_cells = (
                        FrontierSearch.build_new_frontier(
                            mapdata,
                            neighbor,
                            is_frontier,
                            include_frontier_cells,
                        )
                    )
                    if new_frontier.size >= MIN_FRONTIER_SIZE:
                        frontiers.append(new_frontier)
                        if include_frontier_cells:
                            frontier_cells.extend(new_frontier_cells)

        return (FrontierList(frontiers=frontiers), frontier_cells)

    @staticmethod
    def build_new_frontier(
        mapdata: OccupancyGrid,
        initial_cell: "tuple[int, int]",
        is_frontier: dict,
        include_frontier_cells: bool = False,
    ) -> "tuple[Frontier, list[tuple[int, int]]]":
        # Initialize frontier fields
        size = 1
        centroid_x = initial_cell[0]
        centroid_y = initial_cell[1]

        # Create queue for breadth-first search
        queue = []
        queue.append(initial_cell)

        # Initialize list of frontier cells
        frontier_cells = []

        # Breadth-first search for frontier cells
        while queue:
            current = queue.pop(0)

            if include_frontier_cells:
                frontier_cells.append(current)

            for neighbor in PathPlanner.neighbors_of_8(mapdata, current):
                if FrontierSearch.is_new_frontier_cell(mapdata, neighbor, is_frontier):
                    # Mark as frontier
                    is_frontier[neighbor] = True

                    # Update size and centroid
                    size += 1
                    centroid_x += neighbor[0]
                    centroid_y += neighbor[1]
                    queue.append(neighbor)

        # Calculate centroid by taking the average
        centroid_x /= size
        centroid_y /= size

        # Make and return new frontier
        centroid = PathPlanner.grid_to_world(
            mapdata, (int(centroid_x), int(centroid_y))
        )
        return (
            Frontier(size=size, centroid=centroid),
            frontier_cells,
        )

    @staticmethod
    def is_new_frontier_cell(
        mapdata: OccupancyGrid, cell: "tuple[int, int]", is_frontier: dict
    ) -> bool:
        # Cell must be unknown and not already a frontier
        if PathPlanner.get_cell_value(mapdata, cell) != -1 or cell in is_frontier:
            return False

        # Cell should have at least one connected cell that is free
        WALKABLE_THRESHOLD = 50
        for neighbor in PathPlanner.neighbors_of_4(mapdata, cell):
            neighbor_value = PathPlanner.get_cell_value(mapdata, neighbor)
            if neighbor_value >= 0 and neighbor_value < WALKABLE_THRESHOLD:
                return True

        return False