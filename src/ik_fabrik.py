"""
FABRIK (Forward And Backward Reaching Inverse Kinematics) Algorithm
Based on: Aristidou & Lasenby (2011)
"""
import numpy as np


def solve_fabrik(chain, target_position):
    """
    Solve IK using FABRIK method
    
    Args:
        chain: FABRIKChain instance with all necessary attributes
        target_position: (x, y) tuple for target
        
    Returns:
        bool: True if converged within tolerance, False if max iterations reached
        str: "COLLISION_DETECTED" if collision occurs
    """
    target = np.array(target_position, dtype=float)
    sub_joints = chain.joints[:chain.end_effector_index + 1]
    sub_lengths = chain.link_lengths[:chain.end_effector_index]
    total_length = sum(sub_lengths)
    num_sub_joints = len(sub_joints)

    
    # Check if target is reachable
    distance = np.linalg.norm(target - chain.base_position)

    if distance > total_length:
        # Target is unreachable - stretch toward it
        direction = (target - chain.base_position) / distance
        new_joints = np.zeros_like(sub_joints)
        new_joints[0] = chain.base_position
        for i in range(1, num_sub_joints):
            new_joints[i] = new_joints[i-1] + direction * chain.link_lengths[i-1]
        
        # Check for collision
        if chain.detect_collisions(new_joints):
            # Collision detected - don't apply
            return "COLLISION_DETECTED"
        else:
            sub_joints = new_joints
        
        # Apply the stretched configuration
        chain.joints[:chain.end_effector_index + 1] = sub_joints
            
        chain.current_iterations = 0
        return False
    
    # Target is reachable - run FABRIK with obstacle avoidance
    # Use a temporary joints array to compute the solution
    temp_joints = sub_joints.copy()
    iterations = 0
    diff = np.linalg.norm(temp_joints[-1] - target)
    
    # Standard FABRIK loop
    while diff > chain.tolerance and iterations < chain.max_iterations:
        # Forward reaching - start from end effector
        temp_joints[-1] = target

        for i in range(num_sub_joints - 2, -1, -1):
            # Calculate desired next position
            direction = temp_joints[i] - temp_joints[i + 1]
            distance = np.linalg.norm(direction)
            if distance > 0:
                direction = direction / distance
            
            new_pos = temp_joints[i + 1] + direction * chain.link_lengths[i]
            
            temp_joints[i] = new_pos
            
            # Apply joint angle constraints (skip base joint)
            if i > 0:
                temp_joints[i] = chain.apply_joint_constraint(temp_joints, i, i + 1)
        
        # Backward reaching - start from base
        temp_joints[0] = chain.base_position
        
        for i in range(num_sub_joints - 1):
            # Calculate desired next position
            direction = temp_joints[i + 1] - temp_joints[i]
            distance = np.linalg.norm(direction)
            if distance > 0:
                direction = direction / distance
            
            new_pos = temp_joints[i] + direction * chain.link_lengths[i]
            
            temp_joints[i + 1] = new_pos
            
            # Apply joint angle constraints
            if i + 1 > 0:
                temp_joints[i + 1] = chain.apply_joint_constraint(temp_joints, i + 1, i)
        
        diff = np.linalg.norm(temp_joints[-1] - target)
        iterations += 1
    
    chain.current_iterations = iterations
    
    # Apply constraints one more time after FABRIK (in case loop didn't run)
    # This ensures constraints are enforced even if initial pose was already at target
    for i in range(1, num_sub_joints):
        temp_joints[i] = chain.apply_joint_constraint(temp_joints, i, i - 1)
    
    # Check if solution has collision with obstacles
    if chain.detect_collisions(temp_joints):
        # Collision detected - don't apply the solution
        # Keep current position and return collision indicator
        return "COLLISION_DETECTED"
    else:
        # No collision, apply the solution
        sub_joints = temp_joints
    
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
