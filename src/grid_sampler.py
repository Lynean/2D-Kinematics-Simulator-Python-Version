"""
Grid-based sampling for mapping workspace obstacles to joint/configuration space
with collision zone expansion
"""
import numpy as np
from typing import List, Tuple


class GridSampler:
    """
    Grid sampler for mapping obstacles from workspace to configuration space
    Uses uniform grid sampling with collision zone expansion
    """
    
    def __init__(self, chain, grid_resolution=10):
        """
        Initialize Grid sampler
        
        Args:
            chain: FABRIKChain instance
            grid_resolution: Grid spacing in degrees (e.g., 10 = sample every 10 degrees)
        """
        self.chain = chain
        self.grid_resolution = grid_resolution
        self.config_space_obstacles = []
    
    def sample_configuration_space(self, obstacle=None):
        """
        Sample configuration space using grid sampling with collision expansion
        
        Args:
            obstacle: Obstacle instance (not used, checks all obstacles in chain)
            
        Returns:
            Tuple of (collision_configs, non_collision_configs, neighbor_map, endpos_map)
            where neighbor_map is {node_id: [(neighbor_id, distance), ...]}
            and endpos_map is {node_id: [x, y]}
        """
        # Get joint angle limits
        num_joints = self.chain.num_joints - 1  # Exclude end effector
        
        # Create grid samples
        grid_samples = self._create_grid_samples(num_joints)
        
        # Check collisions for all grid points
        collision_configs = []
        non_collision_configs = []
        non_collision_end_positions = []
        collision_indices = []
        
        for idx, angles in enumerate(grid_samples):
            # Convert angles to positions
            joint_positions = self._angles_to_positions(angles)
            
            # Check collision using chain's method
            has_collision = self.chain.detect_collisions(joint_positions)
            
            if has_collision:
                collision_configs.append(angles)
                collision_indices.append(idx)
            else:
                non_collision_configs.append(angles)
                end_pos = joint_positions[-1]
                non_collision_end_positions.append(end_pos)
        
        # Expand collision zones
        expanded_collision_configs, expanded_collision_indices = self._expand_collision_zones(
            grid_samples, collision_indices, num_joints
        )
        
        # Remove expanded collisions from non-collision list
        non_collision_configs_filtered = []
        non_collision_end_positions_filtered = []
        
        expanded_set = set(expanded_collision_indices)
        non_collision_idx = 0
        
        for idx, angles in enumerate(grid_samples):
            if idx not in expanded_set and idx not in collision_indices:
                if non_collision_idx < len(non_collision_configs):
                    if np.allclose(angles, non_collision_configs[non_collision_idx]):
                        non_collision_configs_filtered.append(angles)
                        non_collision_end_positions_filtered.append(non_collision_end_positions[non_collision_idx])
                        non_collision_idx += 1
        
        # Combine original and expanded collisions
        all_collision_configs = collision_configs + expanded_collision_configs
        
        # Build neighbor map
        neighbor_map = self._build_neighbor_map(non_collision_configs_filtered, radius=np.radians(8.0))
        endpos_map = {i: end_pos for i, end_pos in enumerate(non_collision_end_positions_filtered)}
        
        print(f"Grid sampling: {len(grid_samples)} total, {len(collision_configs)} collisions, "
              f"{len(expanded_collision_configs)} expanded, {len(non_collision_configs_filtered)} free")
        
        return all_collision_configs, non_collision_configs_filtered, neighbor_map, endpos_map
    
    def _create_grid_samples(self, num_joints):
        """
        Create uniform grid samples across the configuration space
        
        Args:
            num_joints: Number of joints (excluding end effector)
            
        Returns:
            List of angle configurations (numpy arrays)
        """
        # Create grid points for each joint
        grid_points_per_joint = []
        
        for i in range(num_joints):
            min_angle, max_angle = self.chain.angle_limits[i]
            # Calculate number of steps based on degree spacing
            angle_range_degrees = np.degrees(max_angle - min_angle)
            num_steps = int(angle_range_degrees / self.grid_resolution) + 1
            # Create evenly spaced points across the range
            points = np.linspace(min_angle, max_angle, num_steps)
            grid_points_per_joint.append(points)
        
        # Create meshgrid for all combinations
        if num_joints == 1:
            grid_samples = [np.array([angle]) for angle in grid_points_per_joint[0]]
        elif num_joints == 2:
            mesh = np.meshgrid(grid_points_per_joint[0], grid_points_per_joint[1])
            grid_samples = [np.array([mesh[0].flatten()[i], mesh[1].flatten()[i]]) 
                          for i in range(len(mesh[0].flatten()))]
        elif num_joints == 3:
            mesh = np.meshgrid(grid_points_per_joint[0], grid_points_per_joint[1], 
                             grid_points_per_joint[2])
            grid_samples = [np.array([mesh[0].flatten()[i], mesh[1].flatten()[i], 
                                     mesh[2].flatten()[i]]) 
                          for i in range(len(mesh[0].flatten()))]
        else:
            # General case for more joints
            mesh = np.meshgrid(*grid_points_per_joint)
            flat_meshes = [m.flatten() for m in mesh]
            grid_samples = [np.array([flat_meshes[j][i] for j in range(num_joints)]) 
                          for i in range(len(flat_meshes[0]))]
        
        return grid_samples
    
    def _expand_collision_zones(self, grid_samples, collision_indices, num_joints):
        """
        Expand collision zones by sampling neighbors of collision points
        
        Args:
            grid_samples: All grid samples
            collision_indices: Indices of collision configurations
            num_joints: Number of joints
            
        Returns:
            Tuple of (expanded_collision_configs, expanded_collision_indices)
        """
        # Determine expansion directions based on number of joints
        # 2 joints (excluding end) = 4 directions (each joint ±1 step)
        # 3 joints (excluding end) = 8 directions (combinations)
        
        expanded_collision_configs = []
        expanded_collision_indices = set()
        
        # Get grid step size for each joint
        grid_steps = []
        for i in range(num_joints):
            min_angle, max_angle = self.chain.angle_limits[i]
            step = (max_angle - min_angle) / (self.grid_resolution - 1)
            grid_steps.append(step)
        
        # For each collision point, expand in appropriate directions
        for col_idx in collision_indices:
            col_config = grid_samples[col_idx]
            
            # Generate neighbor offsets
            if num_joints == 2:
                # 4 directions: [+1,0], [-1,0], [0,+1], [0,-1]
                offsets = [
                    np.array([grid_steps[0], 0]),
                    np.array([-grid_steps[0], 0]),
                    np.array([0, grid_steps[1]]),
                    np.array([0, -grid_steps[1]])
                ]
            elif num_joints == 3:
                # 8 directions: all combinations of ±1 for each joint (excluding [0,0,0])
                offsets = []
                for i in [-1, 0, 1]:
                    for j in [-1, 0, 1]:
                        for k in [-1, 0, 1]:
                            if i == 0 and j == 0 and k == 0:
                                continue
                            offset = np.array([i * grid_steps[0], 
                                             j * grid_steps[1], 
                                             k * grid_steps[2]])
                            offsets.append(offset)
            else:
                # General case: all axis-aligned directions (±1 per dimension)
                offsets = []
                for i in range(num_joints):
                    offset_pos = np.zeros(num_joints)
                    offset_pos[i] = grid_steps[i]
                    offsets.append(offset_pos)
                    
                    offset_neg = np.zeros(num_joints)
                    offset_neg[i] = -grid_steps[i]
                    offsets.append(offset_neg)
            
            # Check each neighbor
            for offset in offsets:
                neighbor_config = col_config + offset
                
                # Check if within bounds
                within_bounds = True
                for i in range(num_joints):
                    min_angle, max_angle = self.chain.angle_limits[i]
                    if neighbor_config[i] < min_angle or neighbor_config[i] > max_angle:
                        within_bounds = False
                        break
                
                if within_bounds:
                    # Find if this neighbor exists in grid_samples
                    for idx, sample in enumerate(grid_samples):
                        if np.allclose(sample, neighbor_config, atol=1e-6):
                            if idx not in collision_indices and idx not in expanded_collision_indices:
                                expanded_collision_configs.append(neighbor_config)
                                expanded_collision_indices.add(idx)
                            break
        
        return expanded_collision_configs, list(expanded_collision_indices)
    
    def _angles_to_positions(self, angles):
        """
        Convert joint angles to joint positions
        Uses relative angles (first absolute, rest relative)
        
        Args:
            angles: Array of relative joint angles (radians)
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
    
    def _build_neighbor_map(self, configurations, radius=8.0):
        """
        Build a neighbor map for configurations where each node is connected
        to its nearest neighbors within the specified radius.
        
        Args:
            configurations: List of configuration arrays (joint angles)
            radius: Maximum distance for considering neighbors (in radians)
            
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
            
            # Sort neighbors by distance
            neighbors.sort(key=lambda x: x[1])
            
            neighbor_map[i] = neighbors
        
        return neighbor_map
