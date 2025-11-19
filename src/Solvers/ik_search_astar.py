"""
IK Search using A* pathfinding in configuration space
Finds a path from current configuration to a target end effector position
"""

import numpy as np
from src.astar import AStarPathfinder


class IKSearchAStar:
    """
    IK solver using A* pathfinding in configuration space
    """
    
    def __init__(self, chain, neighbor_map, node_configs, endpos_map):
        """
        Initialize IK search with A* pathfinding
        
        Args:
            chain: FABRIKChain instance
            neighbor_map: Dictionary {node_id: [(neighbor_id, distance), ...]}
            node_configs: List of configuration arrays (joint angles)
            endpos_map: Dictionary {node_id: [x, y]} mapping node to end effector position
        """
        self.chain = chain
        self.neighbor_map = neighbor_map
        self.node_configs = node_configs
        self.endpos_map = endpos_map
        
        # Initialize A* pathfinder
        self.astar = AStarPathfinder()
        self.astar.set_neighbor_map(neighbor_map, node_configs)
    
    def find_path_to_target(self, target_pos, max_attempts=10):
        """
        Find a path from current configuration to target end effector position
        
        Args:
            target_pos: Target end effector position [x, y]
            max_attempts: Maximum number of goal nodes to try (default: 10)
            
        Returns:
            List of angle arrays for each step in the path, or None if no path found
        """
        # Get current configuration from chain
        current_config = self.chain.get_joint_angles()
        
        # Find closest node to current configuration
        start_node = self._find_closest_config_node(current_config)
        if start_node is None:
            print("Could not find start node matching current configuration")
            return None
        
        # Sort endpos_map by distance to target
        sorted_goals = self._sort_by_distance_to_target(target_pos)
        
        # Try pathfinding to each goal in order (up to max_attempts)
        for attempt, (goal_node, goal_pos, distance) in enumerate(sorted_goals[:max_attempts]):
            # Try to find path from start to this goal
            path_configs = self.astar.find_path(start_node, goal_node)
            
            if path_configs is not None:
                print(f"Path found on attempt {attempt + 1}/{max_attempts}")
                print(f"Goal end effector position: [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}]")
                print(f"Distance to target: {distance:.2f}")
                print(f"Path length: {len(path_configs)} steps")
                return path_configs
        
        # No path found after max_attempts
        print(f"No path found after {max_attempts} attempts")
        return None
    
    def _find_closest_config_node(self, current_config):
        """
        Find the node with configuration closest to current configuration
        
        Args:
            current_config: Current joint angles array
            
        Returns:
            Node ID (int) of closest configuration, or None if no configs available
        """
        if len(self.node_configs) == 0:
            return None
        
        min_distance = float('inf')
        closest_node = None
        
        for node_id, config in enumerate(self.node_configs):
            # Calculate Euclidean distance in configuration space
            distance = np.linalg.norm(current_config - config)
            if distance < min_distance:
                min_distance = distance
                closest_node = node_id
        
        return closest_node
    
    def _sort_by_distance_to_target(self, target_pos):
        """
        Sort endpos_map nodes by distance to target position
        
        Args:
            target_pos: Target end effector position [x, y]
            
        Returns:
            List of (node_id, end_pos, distance) tuples sorted by distance (closest first)
        """
        target_array = np.array(target_pos)
        distances = []
        
        for node_id, end_pos in self.endpos_map.items():
            end_pos_array = np.array(end_pos)
            distance = np.linalg.norm(end_pos_array - target_array)
            distances.append((node_id, end_pos, distance))
        
        # Sort by distance (ascending)
        distances.sort(key=lambda x: x[2])
        
        return distances
