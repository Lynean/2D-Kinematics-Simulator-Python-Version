"""
CCD (Cyclic Coordinate Descent) Inverse Kinematics Algorithm
Classic iterative IK solver that rotates each joint individually
"""
import numpy as np


def solve_ccd(chain, target_position):
    """
    Solve IK using CCD (Cyclic Coordinate Descent) method
    
    Args:
        chain: FABRIKChain instance with all necessary attributes
        target_position: (x, y) tuple for target
        
    Returns:
        bool: True if converged within tolerance, False if max iterations reached
        str: "COLLISION_DETECTED" if collision occurs
    """
    target = np.array(target_position, dtype=float)
    sub_joints = chain.joints[:chain.end_effector_index + 1].copy()
    num_sub_joints = len(sub_joints)
    
    # If currently interpolating, don't solve - just update interpolation
    if chain.is_interpolating:
        chain.update_interpolation()
        return True
    
    iterations = 0
    diff = np.linalg.norm(sub_joints[-1] - target)
    
    # CCD algorithm: iterate from end to base
    while diff > chain.tolerance and iterations < chain.max_iterations:
        # For each joint from second-to-last to base
        for i in range(num_sub_joints - 2, -1, -1):
            # Vector from current joint to end effector
            to_end = sub_joints[-1] - sub_joints[i]
            # Vector from current joint to target
            to_target = target - sub_joints[i]
            
            # Calculate angle between vectors
            angle_to_end = np.arctan2(to_end[1], to_end[0])
            angle_to_target = np.arctan2(to_target[1], to_target[0])
            
            # Calculate rotation needed
            rotation = angle_to_target - angle_to_end
            
            # Normalize rotation to [-π, π]
            while rotation > np.pi:
                rotation -= 2 * np.pi
            while rotation < -np.pi:
                rotation += 2 * np.pi
            
            # Rotate all joints from i+1 onwards around joint i
            if abs(rotation) > 0.001:  # Only rotate if significant
                cos_r = np.cos(rotation)
                sin_r = np.sin(rotation)
                
                for j in range(i + 1, num_sub_joints):
                    # Translate to origin
                    relative_pos = sub_joints[j] - sub_joints[i]
                    
                    # Rotate
                    rotated_x = relative_pos[0] * cos_r - relative_pos[1] * sin_r
                    rotated_y = relative_pos[0] * sin_r + relative_pos[1] * cos_r
                    
                    # Translate back
                    sub_joints[j] = sub_joints[i] + np.array([rotated_x, rotated_y])
                
                # Apply joint constraint to the link that was just rotated (if not base)
                if i + 1 < num_sub_joints and i >= 0:
                    sub_joints[i + 1] = chain.apply_joint_constraint(sub_joints, i + 1, i)
        
        # Check convergence
        diff = np.linalg.norm(sub_joints[-1] - target)
        iterations += 1
    
    chain.current_iterations = iterations
    
    # Check if solution has collision with obstacles
    if chain.detect_collisions(sub_joints):
        # Collision detected - don't apply the solution
        return "COLLISION_DETECTED"
    
    # Apply solution
    chain.joints[:chain.end_effector_index + 1] = sub_joints
    
    # For the remaining joints beyond end effector, keep them in line
    if chain.end_effector_index < chain.num_joints - 1:
        for i in range(chain.end_effector_index + 1, chain.num_joints):
            direction = chain.joints[i] - chain.joints[i - 1]
            distance = np.linalg.norm(direction)
            if distance > 0:
                direction = direction / distance
                chain.joints[i] = chain.joints[i - 1] + direction * chain.link_lengths[i - 1]
    
    return diff <= chain.tolerance
