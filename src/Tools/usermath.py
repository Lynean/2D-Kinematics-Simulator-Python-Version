"""
Math utility functions for kinematic chain calculations
"""

import math
import numpy as np


def normalize_angle(angle):
    """Normalize angle to [0, 2π]"""
    return angle % (2 * np.pi)


def wrap_to_pi(angle):
    """Wrap angle to [-π, π]"""
    return np.arctan2(np.sin(angle), np.cos(angle))


def clamp_angle(angle, min_angle, max_angle):
    """Clamp angle to specified limits"""
    angle = normalize_angle(angle)
    
    if angle > max_angle:
        dist_to_max = abs(angle - max_angle)
        dist_to_min = min(abs(angle - min_angle), abs(angle - (min_angle + 2 * np.pi)))
        return max_angle if dist_to_max < dist_to_min else min_angle
    elif angle < min_angle:
        if min_angle - angle > np.pi:
            dist_to_max = abs(angle + 2 * np.pi - max_angle)
            dist_to_min = min_angle - angle
            return max_angle if dist_to_max < dist_to_min else min_angle
        else:
            return min_angle
    
    return angle


def calculate_direction_angle(direction):
    """Calculate angle of a direction vector from horizontal"""
    return np.arctan2(direction[1], direction[0])


def compute_relative_angles(joints):
    """
    Calculate relative angles for each link
    First angle is absolute, subsequent angles are relative to previous link
    """
    if len(joints) < 2:
        return np.array([])
    
    angles = []
    prev_angle = None
    
    for i in range(1, len(joints)):
        direction = joints[i] - joints[i-1]
        angle = calculate_direction_angle(direction)
        
        if prev_angle is None:
            angles.append(angle)
        else:
            relative_angle = wrap_to_pi(angle - prev_angle)
            angles.append(relative_angle)
        
        prev_angle = angle
    
    return np.array(angles)


def compute_joints_from_angles(angles, base_position, link_lengths):
    """Reconstruct joint positions from relative angles"""
    if len(angles) == 0:
        return np.array([base_position])
    
    joints = np.zeros((len(angles) + 1, 2))
    joints[0] = base_position
    
    cumulative_angle = 0.0
    
    for i in range(len(angles)):
        if i == 0:
            cumulative_angle = angles[i]
        else:
            cumulative_angle += angles[i]
        
        direction = np.array([np.cos(cumulative_angle), np.sin(cumulative_angle)])
        joints[i + 1] = joints[i] + direction * link_lengths[i]
    
    return joints


def apply_angle_constraint(joints, joint_index, link_lengths, angle_limits):
    """Apply angle constraint to a joint during IK solving"""
    if joint_index == 0:
        return joints[0]
    
    prev_joint = joints[joint_index - 1]
    current_joint = joints[joint_index]
    
    direction = current_joint - prev_joint
    dist = np.linalg.norm(direction)
    if dist < 1e-6:
        return current_joint
        
    current_angle = calculate_direction_angle(direction)
    
    if joint_index > 1:
        prev_direction = prev_joint - joints[joint_index - 2]
        prev_dist = np.linalg.norm(prev_direction)
        prev_angle = 0 if prev_dist < 1e-6 else calculate_direction_angle(prev_direction)
    else:
        prev_angle = 0
    
    relative_angle = normalize_angle(current_angle - prev_angle)
    limit_index = joint_index - 1
    min_angle, max_angle = angle_limits[limit_index]
    clamped_relative = clamp_angle(relative_angle, min_angle, max_angle)
    
    angle_diff = abs(clamped_relative - relative_angle)
    if angle_diff > np.pi:
        angle_diff = 2 * np.pi - angle_diff
        
    if angle_diff > 0.001:
        new_absolute_angle = prev_angle + clamped_relative
        new_direction = np.array([np.cos(new_absolute_angle), np.sin(new_absolute_angle)])
        link_length = link_lengths[joint_index - 1]
        return prev_joint + new_direction * link_length
    
    return current_joint


def check_segment_circle_intersection(start, end, circle_center, circle_radius):
    """Check if line segment intersects circle"""
    d = end - start
    f = start - circle_center
    
    a = np.dot(d, d)
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - circle_radius * circle_radius
    
    discriminant = b * b - 4 * a * c
    
    if discriminant < 0:
        return False
    
    discriminant = np.sqrt(discriminant)
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)
    
    return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)


# Legacy functions for backward compatibility
def findAngleFromLength(a, b, desired_c):
    """Given two sides of a triangle, find the angle opposite side a"""
    alpha = (desired_c**2 - b**2 - a**2)/(-2*b*a)
    alpha = math.acos(alpha)
    return alpha


def findLengthFromAngle(a, b, angle):
    """Given two sides and an angle, find the hypotenuse"""
    c = math.sqrt(a**2 + b**2 - 2*a*b*math.cos(angle))
    return c


def findAngleBetweenVectors(v1, v2):
    """Find the angle in radians between two vectors"""
    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)
    dot_product = np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)
    angle = np.arccos(dot_product)
    return angle

def findPointFromAngleAndDistance(origin, angle, distance):
    """
    Find a point given an origin, angle, and distance.
    
    Args:
        origin: Origin point (numpy array)
        angle: Angle in radians from the x-axis
        distance: Distance from the origin
    Returns:
        New point (numpy array)
    """
    x = origin[0] + distance * np.cos(angle)
    y = origin[1] + distance * np.sin(angle)
    return np.array([x, y])

def findDistanceBetweenPoints(p1, p2):
    """
    Find the Euclidean distance between two points.
    
    Args:
        p1: First point (numpy array)
        p2: Second point (numpy array)
        
    Returns:
        Distance between p1 and p2
    """
    return np.linalg.norm(p2 - p1)