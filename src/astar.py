"""
A* Pathfinding - Graph-based version
Works with neighbor maps from Monte Carlo configuration space sampling
"""

import numpy as np
from heapq import heappush, heappop


class AStarPathfinder:
    """
    Graph-based A* pathfinding using neighbor maps from Monte Carlo sampling
    """
    
    def __init__(self):
        """
        Initialize A* pathfinder for graph-based pathfinding
        """
        self.neighbor_map = {}  # Dictionary: {node_id: [(neighbor_id, distance), ...]}
        self.node_configs = []  # List of configurations (angles) for each node
        self.path = []  # List of node IDs forming the path
    
    def set_neighbor_map(self, neighbor_map, node_configs):
        """
        Set neighbor map for graph-based pathfinding
        
        Args:
            neighbor_map: Dictionary {node_id: [(neighbor_id, distance), ...]}
            node_configs: List of configurations (joint angles) for each node
        """
        self.neighbor_map = neighbor_map
        self.node_configs = node_configs
        print(f"A* configured with {len(neighbor_map)} nodes")
    
    def heuristic(self, node_a, node_b):
        """
        Calculate heuristic distance between two nodes in configuration space
        
        Args:
            node_a: node_id (int)
            node_b: node_id (int)
            
        Returns:
            float: Estimated distance (Euclidean in config space)
        """
        if isinstance(node_a, int) and isinstance(node_b, int):
            if node_a < len(self.node_configs) and node_b < len(self.node_configs):
                config_a = self.node_configs[node_a]
                config_b = self.node_configs[node_b]
                return np.linalg.norm(config_a - config_b)
        return 0.0
    
    def get_neighbors(self, node_id):
        """
        Get valid neighboring nodes from the neighbor map
        
        Args:
            node_id: Node ID (int)
            
        Returns:
            List of neighboring node IDs
        """
        if isinstance(node_id, int) and node_id in self.neighbor_map:
            # Return just the node IDs, not the distances
            return [neighbor_id for neighbor_id, _ in self.neighbor_map[node_id]]
        return []
    
    def movement_cost(self, from_node, to_node):
        """
        Calculate movement cost between nodes using pre-calculated distance
        
        Args:
            from_node: Starting node ID
            to_node: Destination node ID
            
        Returns:
            float: Movement cost (distance from neighbor map)
        """
        if isinstance(from_node, int) and from_node in self.neighbor_map:
            for neighbor_id, distance in self.neighbor_map[from_node]:
                if neighbor_id == to_node:
                    return distance
        # Default cost if not found
        return 1.0
    
    def find_path(self, start_node, goal_node):
        """
        Find path from start to goal using A* algorithm on graph
        
        Args:
            start_node: Starting node ID (int)
            goal_node: Goal node ID (int)
            
        Returns:
            List of configurations (angle arrays) forming the path, or None if no path found
        """
        # Validate node IDs
        if not isinstance(start_node, int) or not isinstance(goal_node, int):
            print("A* requires integer node IDs!")
            return None
        
        if start_node < 0 or start_node >= len(self.node_configs):
            print(f"Start node {start_node} is out of range!")
            return None
        
        if goal_node < 0 or goal_node >= len(self.node_configs):
            print(f"Goal node {goal_node} is out of range!")
            return None
        
        # A* algorithm
        open_set = []  # Priority queue: (f_score, counter, node_id)
        heappush(open_set, (0, 0, start_node))
        
        came_from = {}  # Best path tracking
        g_score = {start_node: 0}  # Cost from start to each node
        f_score = {start_node: self.heuristic(start_node, goal_node)}  # Estimated total cost
        
        counter = 0  # Tie-breaker for heap
        
        while open_set:
            current_f, _, current = heappop(open_set)
            
            # Goal reached
            if current == goal_node:
                # Reconstruct path (as node IDs)
                path_nodes = [current]
                while current in came_from:
                    current = came_from[current]
                    path_nodes.append(current)
                path_nodes.reverse()
                
                # Convert to configurations and store path
                self.path = path_nodes
                path_configs = [self.node_configs[node_id] for node_id in path_nodes]
                
                return path_configs
            
            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + self.movement_cost(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # This path to neighbor is better
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_node)
                    
                    counter += 1
                    heappush(open_set, (f_score[neighbor], counter, neighbor))
        
        # No path found
        return None
    
    def clear(self):
        """Clear all pathfinding data"""
        self.path = []
