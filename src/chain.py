"""
Simplified FABRIK Kinematic Chain
Uses motion queue system and delegates math to usermath.py
"""

import numpy as np
from src.Solvers.ik_fabrik import solve_fabrik
from src.Solvers.ik_ccd import solve_ccd
from src.Tools import usermath
from src.Tools.astar import AStarPathfinder
from src.Tools.monte_carlo import MonteCarloSampler


class FABRIKChain:
    """Simplified FABRIK Inverse Kinematics Chain"""
    
    def __init__(self, base_position, num_joints=3, link_length=50, obstacles=None):
        """
        Initialize FABRIK chain
        
        Args:
            base_position: (x, y) tuple for base position
            num_joints: Number of joints in the chain
            link_length: Length of each link
            obstacles: List of Obstacle objects
        """
        self.base_position = np.array(base_position, dtype=float)
        self.num_joints = num_joints
        self.link_lengths = [link_length] * (num_joints - 1)
        self.obstacles = obstacles if obstacles is not None else []
        self.end_effector_index = self.num_joints - 1
        
        # Initialize joints in a straight line
        self.joints = np.zeros((num_joints, 2))
        self.joints[0] = self.base_position
        for i in range(1, num_joints):
            self.joints[i] = self.joints[i-1] + np.array([0, link_length])
        
        self.total_length = sum(self.link_lengths)
        self.tolerance = 0.5
        self.max_iterations = 50
        self.current_iterations = 0
        
        # Cached joint angles (relative angles)
        self.joint_angles = usermath.compute_relative_angles(self.joints)
        
        # IK Method selection
        self.ik_method = 'FABRIK'  # 'FABRIK', 'CCD', or 'ASTAR'
        
        # Joint angle constraints (in radians)
        self.angle_limits = [(-np.pi/2, np.pi/2)] * num_joints
        self.angle_limits[0] = (0, np.pi)  # Base joint: 0° to 180°
        
        # Motion queue system
        self.motion_list = []  # Queue of angle configurations to execute
        self.current_motion_index = 0  # Current position in motion_list
        
        # A* pathfinding for IK search
        self.astar = None  # AStarPathfinder instance
        self.neighbor_map = None  # Graph connectivity
        self.node_configs = None  # Sampled configurations
        self.endpos_map = None  # Node ID to end effector position mapping
    
    def set_joint_limits(self, joint_index, min_angle, max_angle):
        """Set angle limits for a specific joint"""
        if 0 <= joint_index < self.num_joints:
            self.angle_limits[joint_index] = (min_angle, max_angle)
    
    def clamp_angle(self, angle, joint_index):
        """Clamp angle to joint's limits"""
        min_angle, max_angle = self.angle_limits[joint_index]
        return usermath.clamp_angle(angle, min_angle, max_angle)
    
    def apply_joint_constraint(self, joints, joint_index, next_joint_index):
        """Apply angle constraint to a joint during FABRIK"""
        return usermath.apply_angle_constraint(joints, joint_index, self.link_lengths, self.angle_limits)
    
    def get_joint_angles(self, joints=None):
        """
        Calculate relative angles for each link
        
        Args:
            joints: Joint positions (uses self.joints if None)
            
        Returns:
            Array of relative angles (first absolute, rest relative)
        """
        if joints is None:
            joints = self.joints
        return usermath.compute_relative_angles(joints)
    
    def _update_joint_angles(self):
        """Update cached joint_angles from current joint positions"""
        if self.num_joints > 1:
            self.joint_angles = self.get_joint_angles()
        else:
            self.joint_angles = np.array([])
    
    def set_joints_from_angles(self, angles, base_position=None):
        """
        Reconstruct joint positions from relative angles
        
        Args:
            angles: Array of relative angles
            base_position: Starting position (uses self.base_position if None)
            
        Returns:
            Array of joint positions
        """
        if base_position is None:
            base_position = self.base_position
        return usermath.compute_joints_from_angles(angles, base_position, self.link_lengths)
    
    def detect_collisions(self, joints=None):
        """
        Detect if any joint or link collides with obstacles
        
        Args:
            joints: Joint positions to check (uses self.joints if None)
            
        Returns:
            bool: True if collision detected
        """
        if joints is None:
            joints = self.joints
            
        if not self.obstacles:
            return False
        
        # Check if any joint (except base) is inside an obstacle
        for i in range(1, len(joints)):
            for obstacle in self.obstacles:
                if obstacle.is_point_inside(joints[i]):
                    return True
        
        # Check if any link intersects an obstacle
        for i in range(len(joints) - 1):
            start = joints[i]
            end = joints[i + 1]
            for obstacle in self.obstacles:
                tangent = obstacle.get_tangent_direction(start, end)
                if tangent is not None:
                    return True
        
        return False
    
    def configure_astar(self, neighbor_map, node_configs, endpos_map):
        """
        Configure A* pathfinding with Monte Carlo sampled data
        
        Args:
            neighbor_map: Dictionary {node_id: [(neighbor_id, distance), ...]}
            node_configs: List of configuration arrays (joint angles)
            endpos_map: Dictionary {node_id: [x, y]}
        """
        self.neighbor_map = neighbor_map
        self.node_configs = node_configs
        self.endpos_map = endpos_map
        
        # Initialize A* pathfinder
        self.astar = AStarPathfinder()
        self.astar.set_neighbor_map(neighbor_map, node_configs)
    
    def configure_astar(self, neighbor_map, node_configs, endpos_map):
        """
        Configure A* pathfinding with Monte Carlo sampled data
        
        Args:
            neighbor_map: Dictionary {node_id: [(neighbor_id, distance), ...]}
            node_configs: List of configuration arrays (joint angles)
            endpos_map: Dictionary {node_id: [x, y]}
        """
        self.neighbor_map = neighbor_map
        self.node_configs = node_configs
        self.endpos_map = endpos_map
        
        # Initialize A* pathfinder
        self.astar = AStarPathfinder()
        self.astar.set_neighbor_map(neighbor_map, node_configs)
    
    def sample_and_configure_astar(self, num_samples=1000):
        """
        Automatically sample configuration space and configure A* pathfinding
        Combines all obstacles into the sampling
        
        Args:
            num_samples: Number of Monte Carlo samples (default: 1000)
        """
        if not self.obstacles:
            print("No obstacles to sample. A* configuration skipped.")
            return
        
        print(f"Sampling configuration space with {num_samples} samples...")
        sampler = MonteCarloSampler(self, num_samples)
        
        # Sample with first obstacle (can be extended to combine multiple obstacles)
        collision, non_collision, neighbor_map, endpos_map = sampler.sample_configuration_space(self.obstacles[0])
        
        print(f"Found {len(non_collision)} collision-free configurations")
        print(f"Found {len(collision)} configurations in collision")
        
        # Configure A* with sampled data
        self.configure_astar(neighbor_map, non_collision, endpos_map)
        print("A* pathfinding configured successfully")
    
    def queue_motion(self, angle_list):
        """
        Queue a list of angle configurations to execute
        
        Args:
            angle_list: List of angle arrays [[angles1], [angles2], ...]
        """
        self.motion_list = angle_list
        self.current_motion_index = 0
    
    def clear_motion_queue(self):
        """Clear the motion queue"""
        self.motion_list = []
        self.current_motion_index = 0
    
    def has_queued_motion(self):
        """Check if there are queued motions to execute"""
        return self.current_motion_index < len(self.motion_list)
    
    def find_path_to_target(self, target_pos, max_attempts=10):
        """
        Find a collision-free path to target position using A* search
        
        Args:
            target_pos: Target end effector position [x, y]
            max_attempts: Maximum number of goal nodes to try (default: 10)
            
        Returns:
            List of angle arrays for each step in the path, or None if no path found
        """
        if self.astar is None:
            print("A* not configured. Call configure_astar() first.")
            return None
        
        # Get current configuration
        current_config = self.get_joint_angles()
        
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
        if self.node_configs is None or len(self.node_configs) == 0:
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
    
    def update(self):
        """
        Execute the next queued motion step
        Updates joint positions from the next angle configuration in motion_list
        
        Returns:
            bool: True if motion step was executed, False if queue is empty
        """
        if not self.has_queued_motion():
            return False
        
        # Get next angle configuration
        angles = self.motion_list[self.current_motion_index]
        
        # Compute joint positions from angles
        self.joints = self.set_joints_from_angles(angles)
        
        # Update cached angles
        self._update_joint_angles()
        
        # Move to next motion step
        self.current_motion_index += 1
        
        return True
    
    def set_ik_method(self, method):
        """Set the IK solving method ('FABRIK', 'CCD', or 'ASTAR')"""
        if method in ['FABRIK', 'CCD', 'ASTAR']:
            self.ik_method = method
        else:
            print(f"Invalid IK method: {method}. Use 'FABRIK', 'CCD', or 'ASTAR'")
    
    def solve(self, target_position):
        """
        Solve IK for target position
        Uses FABRIK, CCD, or A* based on self.ik_method
        
        Args:
            target_position: (x, y) tuple for target
            
        Returns:
            bool: True if converged/path found, False otherwise
        """
        result = None
        
        if self.ik_method == 'ASTAR':
            # Use A* pathfinding to find collision-free path
            path = self.find_path_to_target(target_position, max_attempts=10)
            if path is not None:
                # Queue the path and execute first step
                self.queue_motion(path)
                self.update()  # Execute first step immediately
                result = True
            else:
                result = False
        elif self.ik_method == 'CCD':
            result = solve_ccd(self, target_position)
        else:
            result = solve_fabrik(self, target_position)
        
        # Update cached joint angles after solving
        self._update_joint_angles()
        
        return result
    
    def add_joint(self, link_length=50):
        """Add a new joint to the end of the chain"""
        last_joint = self.joints[-1]
        if len(self.joints) > 1:
            direction = self.joints[-1] - self.joints[-2]
            distance = np.linalg.norm(direction)
            if distance > 0:
                direction = direction / distance
            else:
                direction = np.array([1, 0])
        else:
            direction = np.array([1, 0])
        
        new_joint = last_joint + direction * link_length
        self.joints = np.vstack([self.joints, new_joint])
        self.link_lengths.append(link_length)
        self.num_joints += 1
        self.total_length = sum(self.link_lengths)
        
        # Add default angle limits for new joint
        self.angle_limits.append((-np.pi/2, np.pi/2))
        
        # Update cached angles
        self.joint_angles = np.append(self.joint_angles, 0.0)
        self._update_joint_angles()
    
    def remove_joint(self):
        """Remove the last joint from the chain"""
        if self.num_joints > 1:
            self.joints = self.joints[:-1]
            self.link_lengths.pop()
            self.num_joints -= 1
            self.total_length = sum(self.link_lengths)
            
            # Remove angle limits for removed joint
            if len(self.angle_limits) > 1:
                self.angle_limits.pop()
            
            # Update cached angles
            if self.num_joints > 1:
                self.joint_angles = self.joint_angles[:-1]
                self._update_joint_angles()
            else:
                self.joint_angles = np.array([])
    
    def set_link_length(self, index, length):
        """Set the length of a specific link"""
        if 0 <= index < len(self.link_lengths):
            self.link_lengths[index] = length
            self.total_length = sum(self.link_lengths)

