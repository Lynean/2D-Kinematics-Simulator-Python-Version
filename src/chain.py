import numpy as np

class FABRIKChain:
    """FABRIK Inverse Kinematics Chain"""
    
    def __init__(self, base_position, num_joints=3, link_length=50, obstacles=None):
        """
        Initialize FABRIK chain
        
        Args:
            base_position: (x, y) tuple for base/anchor position
            num_joints: Number of joints in the chain
            link_length: Length of each link
            obstacles: List of Obstacle objects for avoidance
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
            self.joints[i] = self.joints[i-1] + np.array([link_length, 0])
        
        self.total_length = sum(self.link_lengths)
        self.tolerance = 0.5
        self.max_iterations = 20
        self.current_iterations = 0  # Track actual iterations used
        
        # Interpolation system
        self.previous_joints = self.joints.copy()
        self.target_joints = self.joints.copy()
        self.is_interpolating = False
        self.interpolation_progress = 0.0
        self.interpolation_speed = 0.02  # How fast to interpolate (0-1) - slower for smoother animation
        
        # Collision avoidance during interpolation
        self.angle_reversals = {}  # Track which angles are reversed: {joint_index: True/False}
        self.collision_sequence_index = 0  # Current position in collision resolution sequence
        self.last_collision_joint = -1  # Track which joint had collision to generate sequence
    
    def get_joint_angles(self, joints):
        """
        Calculate relative angles for each joint
        
        Args:
            joints: Array of joint positions
            
        Returns:
            Array of angles in radians (relative to previous link)
        """
        angles = []
        for i in range(1, len(joints)):
            direction = joints[i] - joints[i-1]
            angle = np.arctan2(direction[1], direction[0])
            angles.append(angle)
        return np.array(angles)
    
    def set_joints_from_angles(self, angles, base_position):
        """
        Reconstruct joint positions from relative angles
        
        Args:
            angles: Array of angles in radians
            base_position: Starting position
            
        Returns:
            Array of joint positions
        """
        joints = np.zeros((len(angles) + 1, 2))
        joints[0] = base_position
        
        for i in range(len(angles)):
            direction = np.array([np.cos(angles[i]), np.sin(angles[i])])
            joints[i + 1] = joints[i] + direction * self.link_lengths[i]
        
        return joints

    def detect_collisions(self, joints):
        """
        Detect if any joint is in collision with obstacles
        
        Args:
            joints: Joint positions to check
            
        Returns:
            bool: True if collision detected
        """
        if not self.obstacles:
            return False
        
        # Check if any joint (except base) is inside an obstacle's collision radius
        for i in range(1, len(joints)):  # Skip base
            for obstacle in self.obstacles:
                if obstacle.is_point_inside(joints[i]):
                    return True
        #Check if any link intersects an obstacle
        for i in range(len(joints) - 1):
            start = joints[i]
            end = joints[i + 1]
            for obstacle in self.obstacles:
                tangent = obstacle.get_tangent_direction(start, end)
                if tangent is not None:
                    return True
        return False
    
    def update_interpolation(self, poseA = None, poseB = None):
        """
        Update the interpolation between poses with collision-aware angle reversal
        Should be called each frame when is_interpolating is True
        """
        if poseA is not None:
            self.previous_joints = poseA
            self.interpolation_progress = 0.0
            self.is_interpolating = True
        if poseB is not None:
            self.target_joints = poseB
            self.interpolation_progress = 0.0
            self.is_interpolating = True
        if not self.is_interpolating:
            return
        
        # Increment progress
        self.interpolation_progress += self.interpolation_speed
        if self.interpolation_progress >= 1.0:
            # Interpolation complete
            self.joints = self.target_joints
            self.is_interpolating = False
            self.interpolation_progress = 0.0
            self.collision_sequence_index = 0
            self.last_collision_joint = -1
        else:
            # Interpolate using joint angles for smooth servo-like motion
            start_angles = self.get_joint_angles(self.previous_joints)
            target_angles = self.get_joint_angles(self.target_joints)

            # Calculate angle differences
            angle_diffs = target_angles - start_angles
            
            # Wrap to [-π, π] for shortest path
            angle_diffs = np.arctan2(np.sin(angle_diffs), np.cos(angle_diffs))
            
            # Interpolate angles
            current_angles = start_angles + angle_diffs * self.interpolation_progress
            
            # Reconstruct joints from interpolated angles
            temp_joints = self.set_joints_from_angles(current_angles, self.base_position)
            
            # Apply the interpolated joints
            self.joints = temp_joints
    
    def solve(self, target_position):
        """
        Solve FABRIK IK for target position with obstacle avoidance
        
        Args:
            target_position: (x, y) tuple for target
            
        Returns:
            bool: True if converged within tolerance, False if max iterations reached
        """
        target = np.array(target_position, dtype=float)
        sub_joints = self.joints[:self.end_effector_index + 1]
        sub_lengths = self.link_lengths[:self.end_effector_index]
        total_length = sum(sub_lengths)
        num_sub_joints = len(sub_joints)
        # If currently interpolating, don't solve - just update interpolation
        if self.is_interpolating:
            self.update_interpolation()
            return True
        
        # Check if target is reachable
        distance = np.linalg.norm(target - self.base_position)

        if distance > total_length:
            # Target is unreachable - stretch toward it
            direction = (target - self.base_position) / distance
            new_joints = np.zeros_like(sub_joints)
            new_joints[0] = self.base_position
            for i in range(1, num_sub_joints):
                new_joints[i] = new_joints[i-1] + direction * self.link_lengths[i-1]
            
            # Check for collision
            if self.detect_collisions(new_joints):
                # Collision detected - don't apply
                return "COLLISION_DETECTED"
            else:
                sub_joints = new_joints
                
            self.current_iterations = 0
            return False
        
        # Target is reachable - run FABRIK with obstacle avoidance
        # Use a temporary joints array to compute the solution
        temp_joints = sub_joints.copy()
        iterations = 0
        diff = np.linalg.norm(temp_joints[-1] - target)
        
        while diff > self.tolerance and iterations < self.max_iterations:
            # Forward reaching - start from end effector
            temp_joints[-1] = target

            for i in range(num_sub_joints - 2, -1, -1):
                # Calculate desired next position
                direction = temp_joints[i] - temp_joints[i + 1]
                distance = np.linalg.norm(direction)
                if distance > 0:
                    direction = direction / distance
                
                new_pos = temp_joints[i + 1] + direction * self.link_lengths[i]
                
                
                temp_joints[i] = new_pos
            
            # Backward reaching - start from base
            temp_joints[0] = self.base_position
            
            for i in range(num_sub_joints - 1):
                # Calculate desired next position
                direction = temp_joints[i + 1] - temp_joints[i]
                distance = np.linalg.norm(direction)
                if distance > 0:
                    direction = direction / distance
                
                new_pos = temp_joints[i] + direction * self.link_lengths[i]
                
                temp_joints[i + 1] = new_pos
                
            
            diff = np.linalg.norm(temp_joints[-1] - target)
            iterations += 1
        
        self.current_iterations = iterations
        
        # Check if solution has collision with obstacles
        if self.detect_collisions(temp_joints):
            # Collision detected - don't apply the solution
            # Keep current position and return collision indicator
            return "COLLISION_DETECTED"
        else:
            # No collision, apply the solution
            sub_joints = temp_joints
        self.joints[:self.end_effector_index + 1] = sub_joints
        #For the remaining joints beyond end effector, keep them in line
        if self.end_effector_index < self.num_joints - 1:
                for i in range(self.end_effector_index + 1, self.num_joints):
                    direction = self.joints[i] - self.joints[i - 1]
                    distance = np.linalg.norm(direction)
                    if distance > 0:
                        direction = direction / distance
                        self.joints[i] = self.joints[i - 1] + direction * self.link_lengths[i - 1]
        return diff <= self.tolerance
    
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
    
    def remove_joint(self):
        """Remove the last joint from the chain"""
        if self.num_joints > 2:
            self.joints = self.joints[:-1]
            self.link_lengths.pop()
            self.num_joints -= 1
            self.total_length = sum(self.link_lengths)
    
    def set_link_length(self, index, length):
        """Set the length of a specific link"""
        if 0 <= index < len(self.link_lengths):
            self.link_lengths[index] = length
            self.total_length = sum(self.link_lengths)

