"""
Path management for FABRIK simulator
Handles path drawing, interpolation, and following logic
"""

import numpy as np


class Path:
    """Manages path drawing, interpolation, and following for the FABRIK chain"""
    
    def __init__(self, step_size=10.0, speed=2.0):
        """
        Initialize path manager
        
        Args:
            step_size: Distance between interpolated points
            speed: Movement speed when following path
        """
        # Path data
        self.raw_points = []  # Raw clicked points
        self.interpolated_points = []  # Interpolated smooth path
        
        # Path state
        self.is_drawing = False
        self.is_following = False
        self.current_index = 0
        
        # Path parameters
        self.step_size = step_size
        self.speed = speed
    
    def start_drawing(self):
        """Start path drawing mode"""
        self.is_drawing = True
        self.raw_points = []
        self.interpolated_points = []
        self.is_following = False
    
    def stop_drawing(self):
        """Stop path drawing mode"""
        self.is_drawing = False
    
    def add_point(self, point):
        """
        Add a point to the path
        
        Args:
            point: [x, y] coordinates
        """
        self.raw_points.append(point)
        self.interpolate()
    
    def clear(self):
        """Clear all path data"""
        self.raw_points = []
        self.interpolated_points = []
        self.current_index = 0
        self.is_following = False
    
    def interpolate(self):
        """Interpolate between raw points to create smooth path"""
        if len(self.raw_points) == 0:
            self.interpolated_points = []
            return
        
        if len(self.raw_points) == 1:
            self.interpolated_points = [self.raw_points[0]]
            return
        
        # Interpolate between consecutive points
        interpolated = []
        
        for i in range(len(self.raw_points)):
            start_point = np.array(self.raw_points[i])
            interpolated.append(start_point.tolist())
            
            # Interpolate to next point (or back to first if this is the last point)
            if i < len(self.raw_points) - 1:
                end_point = np.array(self.raw_points[i + 1])
            else:
                # Close the loop back to first point
                end_point = np.array(self.raw_points[0])
            
            # Calculate distance and number of steps
            distance = np.linalg.norm(end_point - start_point)
            if distance > self.step_size:
                num_steps = int(distance / self.step_size)
                
                for step in range(1, num_steps):
                    t = step / num_steps
                    interp_point = start_point + t * (end_point - start_point)
                    interpolated.append(interp_point.tolist())
        
        self.interpolated_points = interpolated
    
    def start_following(self):
        """Start following the path"""
        if len(self.interpolated_points) > 0:
            self.is_following = True
            self.current_index = 0
    
    def stop_following(self):
        """Stop following the path"""
        self.is_following = False
    
    def get_current_target(self):
        """
        Get the current target position on the path
        
        Returns:
            numpy array of [x, y] or None if no path
        """
        if len(self.interpolated_points) == 0:
            return None
        return np.array(self.interpolated_points[self.current_index])
    
    def advance_to_next_waypoint(self):
        """Move to the next waypoint in the path"""
        if len(self.interpolated_points) > 0:
            self.current_index = (self.current_index + 1) % len(self.interpolated_points)
    
    def is_point_in_reach(self, point, base_position, reach_radius):
        """
        Check if a point is within reach of the base
        
        Args:
            point: [x, y] coordinates to check
            base_position: Base position of the chain
            reach_radius: Maximum reach distance
            
        Returns:
            bool: True if point is within reach
        """
        distance = np.linalg.norm(np.array(point) - base_position)
        return distance <= reach_radius
    
    def has_points(self):
        """Check if path has any points"""
        return len(self.interpolated_points) > 0
    
    def set_step_size(self, step_size):
        """
        Update step size and re-interpolate
        
        Args:
            step_size: New step size value
        """
        self.step_size = step_size
        if len(self.raw_points) > 0:
            self.interpolate()
    
    def set_speed(self, speed):
        """
        Update path following speed
        
        Args:
            speed: New speed value
        """
        self.speed = speed
    
    def get_first_waypoint(self):
        """
        Get the first waypoint of the path
        
        Returns:
            numpy array of [x, y] or None if no path
        """
        if len(self.interpolated_points) > 0:
            return np.array(self.interpolated_points[0])
        return None
