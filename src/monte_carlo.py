"""
Monte Carlo sampling for mapping workspace obstacles to joint/configuration space
"""
import numpy as np
from typing import List, Tuple


class MonteCarloSampler:
    """
    Monte Carlo sampler for mapping obstacles from workspace to configuration space
    """
    
    def __init__(self, chain, num_samples=1000):
        """
        Initialize Monte Carlo sampler
        
        Args:
            chain: FABRIKChain instance
            num_samples: Number of random samples to generate
        """
        self.chain = chain
        self.num_samples = num_samples
        self.config_space_obstacles = []  # List of (angles, collision) tuples
    
    def sample_configuration_space(self, obstacle):
        """
        Sample configuration space to find which configurations cause collisions with obstacle
        
        Args:
            obstacle: Obstacle instance
            
        Returns:
            Tuple of (collision_configs, non_collision_configs, neighbor_map, endpos_map)
            where neighbor_map is {node_id: [(neighbor_id, distance), ...]}
            and endpos_map is {node_id: [x, y]}
        """
        collision_configs = []
        non_collision_configs = []
        non_collision_end_positions = []  # Store end effector positions
        
        # Get joint angle limits
        num_joints = self.chain.num_joints - 1  # Exclude end effector
        
        for _ in range(self.num_samples):
            # Generate random joint angles within limits
            random_angles = []
            for i in range(num_joints):
                min_angle, max_angle = self.chain.angle_limits[i]
                angle = np.random.uniform(min_angle, max_angle)
                random_angles.append(angle)
            
            # Set chain to this configuration
            joint_positions = self._angles_to_positions(random_angles)
            
            # Check if this configuration collides with obstacle
            # Use the chain's detect_collisions method for consistency
            has_collision = self.chain.detect_collisions(joint_positions)
            
            if has_collision:
                collision_configs.append(np.array(random_angles))
            else:
                non_collision_configs.append(np.array(random_angles))
                # Store end effector position (last position)
                end_pos = joint_positions[-1]
                non_collision_end_positions.append(end_pos)
        
        # Build neighbor map and endpos map for non-collision configurations
        # Radius in radians (8 degrees = ~0.1396 radians)
        neighbor_map = self._build_neighbor_map(non_collision_configs, radius=np.radians(8.0))
        endpos_map = {i: end_pos for i, end_pos in enumerate(non_collision_end_positions)}
        
        return collision_configs, non_collision_configs, neighbor_map, endpos_map
    
    def _angles_to_positions(self, angles):
        """
        Convert joint angles to joint positions
        Uses relative angles (first absolute, rest relative)
        
        Args:
            angles: List of relative joint angles (radians)
                   First angle is absolute, rest are relative to previous link
            
        Returns:
            Array of joint positions
        """
        if len(angles) == 0:
            return np.array([self.chain.base_position])
        
        positions = np.zeros((len(angles) + 1, 2))
        positions[0] = self.chain.base_position
        
        cumulative_angle = 0.0
        
        for i, angle in enumerate(angles):
            if i == 0:
                # First angle is absolute
                cumulative_angle = angle
            else:
                # Add relative angle to cumulative
                cumulative_angle += angle
            
            # Position next joint
            direction = np.array([np.cos(cumulative_angle), np.sin(cumulative_angle)])
            positions[i + 1] = positions[i] + direction * self.chain.link_lengths[i]
        
        return positions
    
    def _check_collision(self, positions, obstacle):
        """
        Check if any link segment collides with obstacle
        Uses collision_radius (not warning radius) to match chain's collision detection
        
        Args:
            positions: Array of joint positions
            obstacle: Obstacle instance
            
        Returns:
            True if collision detected
        """
        # Check ALL joints (including base) are inside obstacle
        for pos in positions:
            if obstacle.is_point_inside(pos):
                return True
        
        # Check each link segment intersects obstacle
        for i in range(len(positions) - 1):
            start = positions[i]
            end = positions[i + 1]
            
            # Check if segment intersects obstacle (using collision_radius)
            if self._segment_intersects_circle(start, end, obstacle.position, 
                                               obstacle.collision_radius):
                return True
        
        return False
    
    def _segment_intersects_circle(self, start, end, circle_center, circle_radius):
        """
        Check if line segment intersects circle
        
        Args:
            start: Segment start point
            end: Segment end point
            circle_center: Circle center
            circle_radius: Circle radius
            
        Returns:
            True if intersection exists
        """
        # First check if either endpoint is inside the circle
        dist_start = np.linalg.norm(start - circle_center)
        dist_end = np.linalg.norm(end - circle_center)
        
        if dist_start <= circle_radius or dist_end <= circle_radius:
            return True
        
        # Vector from start to end
        d = end - start
        # Vector from start to circle center
        f = start - circle_center
        
        a = np.dot(d, d)
        
        # Handle degenerate case (zero-length segment)
        if a < 1e-10:
            return False  # Already checked endpoints above
        
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - circle_radius * circle_radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return False
        
        # Check if intersection points are within segment
        discriminant = np.sqrt(discriminant)
        
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        
        # Check if either intersection is within (0, 1) - exclusive since we already checked endpoints
        return (0 < t1 < 1) or (0 < t2 < 1)
    
    def _build_neighbor_map(self, configurations, radius=8.0):
        """
        Build a neighbor map for configurations where each node is connected
        to its nearest neighbors within the specified radius.
        
        Args:
            configurations: List of configuration arrays (joint angles)
            radius: Maximum distance for considering neighbors
            
        Returns:
            Dictionary mapping node_id to list of (neighbor_id, distance) tuples
        """
        neighbor_map = {}
        
        if len(configurations) == 0:
            return neighbor_map
        
        # Convert to numpy array for efficient distance calculations
        configs_array = np.array(configurations)
        
        # Build neighbor map for each configuration
        for i in range(len(configurations)):
            neighbors = []
            
            # Find all neighbors within radius
            for j in range(len(configurations)):
                if i == j:
                    continue
                
                # Calculate Euclidean distance in configuration space
                distance = np.linalg.norm(configs_array[i] - configs_array[j])
                
                # If within radius, add as neighbor
                if distance <= radius:
                    neighbors.append((j, float(distance)))
            
            # Sort neighbors by distance (optional, but helpful)
            neighbors.sort(key=lambda x: x[1])
            
            neighbor_map[i] = neighbors
        
        return neighbor_map
    
    def get_obstacle_region_2d(self, obstacle, joint_indices=(0, 1)):
        """
        Get obstacle region in 2D configuration space (for first two joints)
        
        Args:
            obstacle: Obstacle instance
            joint_indices: Which two joints to consider (default: 0, 1)
            
        Returns:
            List of (angle0, angle1) tuples representing collision configurations
        """
        collision_configs, non_collision_configs, neighbor_map = self.sample_configuration_space(obstacle)
        
        if len(collision_configs) == 0:
            return []
        
        # Extract specified joint angles
        collision_2d = []
        for config in collision_configs:
            if len(config) > max(joint_indices):
                angle0 = config[joint_indices[0]]
                angle1 = config[joint_indices[1]]
                collision_2d.append((angle0, angle1))
        
        return collision_2d
    
    def visualize_config_space_obstacle(self, obstacle):
        """
        Generate visualization data for configuration space obstacle
        
        Args:
            obstacle: Obstacle instance
            
        Returns:
            Dictionary with visualization data
        """
        collision_configs, non_collision_configs, neighbor_map, endpos_map = self.sample_configuration_space(obstacle)
        collision_2d = self.get_obstacle_region_2d(obstacle)
        
        return {
            'total_samples': self.num_samples,
            'collision_count': len(collision_configs),
            'collision_percentage': (len(collision_configs) / self.num_samples) * 100,
            'collision_configs': collision_configs,
            'non_collision_configs': non_collision_configs,
            'neighbor_map': neighbor_map,
            'endpos_map': endpos_map,
            'collision_2d': collision_2d
        }
