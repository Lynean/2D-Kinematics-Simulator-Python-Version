"""
A* Pathfinding for FABRIK simulator
Handles grid-based pathfinding with obstacle avoidance
"""

import numpy as np
from heapq import heappush, heappop


class AStarPathfinder:
    """A* pathfinding on a grid with configurable resolution and heuristic"""
    
    def __init__(self, grid_size=10):
        """
        Initialize A* pathfinder
        
        Args:
            grid_size: Size of each grid cell in pixels
        """
        self.grid_size = grid_size
        self.grid_costs = {}  # Dictionary: (grid_x, grid_y) -> cost
        self.grid_obstacles = set()  # Set of (grid_x, grid_y) that are obstacles
        self.path = []  # List of grid coordinates forming the path
        self.workspace_bounds = (0, 0, 600, 600)  # (min_x, min_y, max_x, max_y)
    
    def set_grid_size(self, size):
        """Update grid size and clear cached data"""
        self.grid_size = size
        self.clear()
    
    def set_workspace_bounds(self, min_x, min_y, max_x, max_y):
        """Set the workspace boundaries for grid generation"""
        self.workspace_bounds = (min_x, min_y, max_x, max_y)
    
    def world_to_grid(self, x, y):
        """
        Convert world coordinates to grid coordinates
        
        Args:
            x, y: World coordinates in pixels
            
        Returns:
            (grid_x, grid_y): Grid cell indices
        """
        grid_x = int(round(x / self.grid_size))
        grid_y = int(round(y / self.grid_size))
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x, grid_y):
        """
        Convert grid coordinates to world coordinates (cell center)
        
        Args:
            grid_x, grid_y: Grid cell indices
            
        Returns:
            (x, y): World coordinates in pixels
        """
        x = grid_x * self.grid_size
        y = grid_y * self.grid_size
        return (x, y)
    
    def heuristic(self, grid_a, grid_b):
        """
        Calculate heuristic distance between two grid cells
        Easily replaceable for different heuristics
        
        Args:
            grid_a: (grid_x, grid_y) tuple
            grid_b: (grid_x, grid_y) tuple
            
        Returns:
            float: Estimated distance
        """
        # Euclidean distance (can be swapped for Manhattan, etc.)
        dx = grid_a[0] - grid_b[0]
        dy = grid_a[1] - grid_b[1]
        return np.sqrt(dx * dx + dy * dy)
    
    def get_neighbors(self, grid_pos):
        """
        Get valid neighboring grid cells (8-directional)
        
        Args:
            grid_pos: (grid_x, grid_y) tuple
            
        Returns:
            List of neighboring (grid_x, grid_y) tuples
        """
        grid_x, grid_y = grid_pos
        neighbors = []
        
        # 8-directional movement
        directions = [
            (-1, -1), (0, -1), (1, -1),  # Top row
            (-1,  0),          (1,  0),  # Middle row
            (-1,  1), (0,  1), (1,  1)   # Bottom row
        ]
        
        for dx, dy in directions:
            neighbor = (grid_x + dx, grid_y + dy)
            
            # Check if neighbor is not an obstacle
            if neighbor not in self.grid_obstacles:
                neighbors.append(neighbor)
        
        return neighbors
    
    def movement_cost(self, from_grid, to_grid):
        """
        Calculate movement cost between adjacent grid cells
        
        Args:
            from_grid: (grid_x, grid_y) starting cell
            to_grid: (grid_x, grid_y) destination cell
            
        Returns:
            float: Movement cost (diagonal = sqrt(2), straight = 1)
        """
        dx = abs(to_grid[0] - from_grid[0])
        dy = abs(to_grid[1] - from_grid[1])
        
        # Diagonal movement costs more
        if dx + dy == 2:
            return np.sqrt(2)
        else:
            return 1.0
    
    def generate_obstacle_grid(self, obstacles):
        """
        Generate grid obstacles from obstacle list
        A grid cell is an obstacle if its center is inside the warning radius
        
        Args:
            obstacles: List of Obstacle objects
        """
        self.grid_obstacles.clear()
        
        if not obstacles:
            return
        
        min_x, min_y, max_x, max_y = self.workspace_bounds
        
        # Iterate through all grid cells in workspace
        for grid_x in range(int(min_x / self.grid_size), int(max_x / self.grid_size) + 1):
            for grid_y in range(int(min_y / self.grid_size), int(max_y / self.grid_size) + 1):
                # Get cell center in world coordinates
                world_x, world_y = self.grid_to_world(grid_x, grid_y)
                
                # Check if this cell center is inside any obstacle's warning radius
                for obstacle in obstacles:
                    distance = np.linalg.norm(
                        np.array([world_x, world_y]) - obstacle.position
                    )
                    
                    # Use warning radius (not collision radius)
                    if distance <= obstacle.radius:
                        self.grid_obstacles.add((grid_x, grid_y))
                        break
    
    def find_path(self, start_world, goal_world):
        """
        Find path from start to goal using A* algorithm
        
        Args:
            start_world: (x, y) world coordinates of start
            goal_world: (x, y) world coordinates of goal
            
        Returns:
            List of (x, y) world coordinates forming the path, or None if no path found
        """
        # Convert to grid coordinates
        start = self.world_to_grid(start_world[0], start_world[1])
        goal = self.world_to_grid(goal_world[0], goal_world[1])
        
        # Check if start or goal is an obstacle
        if start in self.grid_obstacles:
            print("Start position is inside an obstacle!")
            return None
        
        if goal in self.grid_obstacles:
            print("Goal position is inside an obstacle!")
            return None
        
        # A* algorithm
        open_set = []  # Priority queue: (f_score, counter, grid_pos)
        heappush(open_set, (0, 0, start))
        
        came_from = {}  # Best path tracking
        g_score = {start: 0}  # Cost from start to each node
        f_score = {start: self.heuristic(start, goal)}  # Estimated total cost
        
        counter = 0  # Tie-breaker for heap
        
        while open_set:
            current_f, _, current = heappop(open_set)
            
            # Goal reached
            if current == goal:
                # Reconstruct path
                path_grid = [current]
                while current in came_from:
                    current = came_from[current]
                    path_grid.append(current)
                path_grid.reverse()
                
                # Convert to world coordinates
                self.path = path_grid
                world_path = [self.grid_to_world(gx, gy) for gx, gy in path_grid]
                
                # Store costs for visualization
                self.grid_costs = dict(g_score)
                
                return world_path
            
            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + self.movement_cost(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # This path to neighbor is better
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    
                    counter += 1
                    heappush(open_set, (f_score[neighbor], counter, neighbor))
        
        # No path found
        self.grid_costs = dict(g_score)  # Store costs anyway for visualization
        return None
    
    def get_grid_cost(self, grid_x, grid_y):
        """
        Get the cost value for a specific grid cell
        
        Args:
            grid_x, grid_y: Grid cell coordinates
            
        Returns:
            float: Cost value, or None if not visited
        """
        return self.grid_costs.get((grid_x, grid_y), None)
    
    def is_obstacle(self, grid_x, grid_y):
        """
        Check if a grid cell is an obstacle
        
        Args:
            grid_x, grid_y: Grid cell coordinates
            
        Returns:
            bool: True if obstacle
        """
        return (grid_x, grid_y) in self.grid_obstacles
    
    def clear(self):
        """Clear all pathfinding data"""
        self.grid_costs.clear()
        self.grid_obstacles.clear()
        self.path = []
    
    def get_all_grid_cells_in_bounds(self):
        """
        Get all grid cells within workspace bounds
        
        Returns:
            List of (grid_x, grid_y) tuples
        """
        min_x, min_y, max_x, max_y = self.workspace_bounds
        cells = []
        
        for grid_x in range(int(min_x / self.grid_size), int(max_x / self.grid_size) + 1):
            for grid_y in range(int(min_y / self.grid_size), int(max_y / self.grid_size) + 1):
                cells.append((grid_x, grid_y))
        
        return cells
