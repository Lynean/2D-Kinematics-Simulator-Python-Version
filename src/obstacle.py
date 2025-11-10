import numpy as np

class Obstacle:
    """Obstacle point with warning zone for avoidance"""
    
    def __init__(self, position, radius=50.0, collision_radius=None):
        """
        Initialize obstacle
        
        Args:
            position: (x, y) tuple for obstacle center
            radius: Warning zone radius
            collision_radius: Hard collision radius (defaults to radius * 0.6)
        """
        self.position = np.array(position, dtype=float)
        self.radius = radius
        self.collision_radius = collision_radius if collision_radius is not None else radius * 0.6
    
    def is_point_inside(self, point):
        """Check if point is inside collision circle"""
        distance = np.linalg.norm(point - self.position)
        return distance <= self.collision_radius
    
    def get_nearest_boundary_point(self, point):
        """Get nearest point on collision boundary"""
        to_point = point - self.position
        distance = np.linalg.norm(to_point)
        if distance < 0.001:
            # Point at center, push in arbitrary direction
            return self.position + np.array([self.collision_radius, 0])
        direction = to_point / distance
        return self.position + direction * self.collision_radius
    
    def get_tangent_direction(self, start, end):
        """
        Get tangent direction for a line that would pass through obstacle
        
        Args:
            start: Line start point
            end: Line end point
            
        Returns:
            Tangent direction vector (normalized) or None if no collision
        """
        # Vector from start to end
        line_vec = end - start
        line_length = np.linalg.norm(line_vec)
        if line_length < 0.001:
            return None
        
        line_dir = line_vec / line_length
        
        # Vector from start to obstacle center
        to_obstacle = self.position - start
        
        # Project obstacle center onto line
        projection_length = np.dot(to_obstacle, line_dir)
        
        # Check if projection is within line segment
        if projection_length < 0 or projection_length > line_length:
            return None
        
        # Closest point on line to obstacle
        closest_point = start + line_dir * projection_length
        
        # Distance from obstacle center to line
        distance_to_line = np.linalg.norm(self.position - closest_point)
        
        # Check if line intersects collision circle
        if distance_to_line >= self.collision_radius:
            return None
        
        # Calculate tangent direction
        to_center = self.position - start
        to_center_norm = np.linalg.norm(to_center)
        if to_center_norm < 0.001:
            return None
        
        to_center_dir = to_center / to_center_norm
        
        # Two possible tangent directions
        tangent1 = np.array([-to_center_dir[1], to_center_dir[0]])
        tangent2 = np.array([to_center_dir[1], -to_center_dir[0]])
        
        # Choose tangent more aligned with original direction
        if np.dot(tangent1, line_dir) > np.dot(tangent2, line_dir):
            return tangent1
        else:
            return tangent2


