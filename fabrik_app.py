"""
FABRIK (Forward And Backward Reaching Inverse Kinematics) Interactive Application
Features:
- Interactive joint/link creation
- Real-time FABRIK IK solver
- Target manipulation
- Visual feedback
"""

import sys
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QPushButton, QLabel, QSpinBox, 
                              QDoubleSpinBox, QGroupBox, QSlider, QCheckBox,
                              QDialog, QTableWidget, QTableWidgetItem, QHeaderView,
                              QScrollArea)
from PyQt6.QtCore import Qt, QTimer
import pyqtgraph as pg


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
        return distance < self.collision_radius
    
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
    
    def get_avoidance_adjustment(self, point, target_direction):
        """
        Calculate adjustment to movement direction to avoid obstacle
        
        Args:
            point: Current position that might need adjustment
            target_direction: Normalized direction toward target
            
        Returns:
            Adjusted normalized direction vector
        """
        # Vector from obstacle to point
        to_point = point - self.position
        distance = np.linalg.norm(to_point)
        
        # If outside warning zone, no adjustment needed
        if distance >= self.radius:
            return target_direction
        
        # Calculate penetration factor (0 at edge, 1 at center)
        if distance < 0.1:  # Avoid division by zero at center
            distance = 0.1
        
        penetration = (self.radius - distance) / self.radius
        
        # Radial direction (away from obstacle)
        radial = to_point / distance
        
        # Tangent direction (perpendicular to radial)
        # Choose tangent that's most aligned with target direction
        tangent1 = np.array([-radial[1], radial[0]])
        tangent2 = np.array([radial[1], -radial[0]])
        
        # Pick tangent more aligned with target direction
        if np.dot(tangent1, target_direction) > np.dot(tangent2, target_direction):
            tangent = tangent1
        else:
            tangent = tangent2
        
        # Blend between target direction and tangent based on penetration
        # Also add some radial component to push away from obstacle
        adjusted = (1 - penetration) * target_direction + penetration * (0.7 * tangent + 0.3 * radial)
        
        # Normalize
        adjusted_norm = np.linalg.norm(adjusted)
        if adjusted_norm > 0:
            adjusted = adjusted / adjusted_norm
        
        return adjusted


class JointConfigDialog(QDialog):
    """Dialog for configuring joint positions"""
    
    def __init__(self, chain, parent=None):
        super().__init__(parent)
        self.chain = chain
        self.setWindowTitle("Joint Configuration")
        self.setGeometry(200, 200, 500, 600)
        
        self.setup_ui()
        self.load_joint_data()
    
    def setup_ui(self):
        """Setup the dialog UI"""
        layout = QVBoxLayout(self)
        
        # Instructions
        instructions = QLabel(
            "<b>Configure Joint Positions</b><br>"
            "Modify X and Y coordinates for each joint.<br>"
            "Link lengths will be maintained when solving."
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        
        # Table for joint positions
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Joint", "X Position", "Y Position"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        layout.addWidget(self.table)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        apply_btn = QPushButton("Apply")
        apply_btn.clicked.connect(self.apply_changes)
        button_layout.addWidget(apply_btn)
        
        reset_btn = QPushButton("Reset")
        reset_btn.clicked.connect(self.load_joint_data)
        button_layout.addWidget(reset_btn)
        
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        button_layout.addWidget(close_btn)
        
        layout.addLayout(button_layout)
        
        # Info label
        self.info_label = QLabel()
        self.info_label.setStyleSheet("color: gray; font-style: italic;")
        layout.addWidget(self.info_label)
    
    def load_joint_data(self):
        """Load current joint positions into table"""
        self.table.setRowCount(self.chain.num_joints)
        
        for i in range(self.chain.num_joints):
            # Joint number
            joint_item = QTableWidgetItem(f"Joint {i}")
            joint_item.setFlags(joint_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            if i == 0:
                joint_item.setText(f"Joint {i} (Base)")
            self.table.setItem(i, 0, joint_item)
            
            # X position
            x_item = QTableWidgetItem(f"{self.chain.joints[i][0]:.2f}")
            self.table.setItem(i, 1, x_item)
            
            # Y position
            y_item = QTableWidgetItem(f"{self.chain.joints[i][1]:.2f}")
            self.table.setItem(i, 2, y_item)
        
        self.update_info()
    
    def apply_changes(self):
        """Apply changes from table to chain"""
        try:
            new_positions = []
            for i in range(self.chain.num_joints):
                x = float(self.table.item(i, 1).text())
                y = float(self.table.item(i, 2).text())
                new_positions.append([x, y])
            
            # Update chain joints
            self.chain.joints = np.array(new_positions)
            
            # Update base position
            self.chain.base_position = self.chain.joints[0].copy()
            
            # Recalculate link lengths based on new positions
            for i in range(len(self.chain.link_lengths)):
                length = np.linalg.norm(self.chain.joints[i + 1] - self.chain.joints[i])
                self.chain.link_lengths[i] = length
            
            self.chain.total_length = sum(self.chain.link_lengths)
            
            self.info_label.setText("✓ Changes applied successfully!")
            self.info_label.setStyleSheet("color: green; font-style: italic;")
            
        except ValueError as e:
            self.info_label.setText(f"✗ Error: Invalid number format!")
            self.info_label.setStyleSheet("color: red; font-style: italic;")
        except Exception as e:
            self.info_label.setText(f"✗ Error: {str(e)}")
            self.info_label.setStyleSheet("color: red; font-style: italic;")
    
    def update_info(self):
        """Update info label with chain statistics"""
        total_length = sum(self.chain.link_lengths)
        self.info_label.setText(f"Total chain length: {total_length:.2f}")
        self.info_label.setStyleSheet("color: gray; font-style: italic;")


class JointAnglesDialog(QDialog):
    """Dialog for displaying real-time joint angles"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_widget = parent
        self.setWindowTitle("Joint Angles Monitor")
        self.setGeometry(250, 250, 400, 500)
        
        self.setup_ui()
        
        # Timer to update angles
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_angles)
        self.timer.start(50)  # Update at 20 Hz
    
    def setup_ui(self):
        """Setup the dialog UI"""
        layout = QVBoxLayout(self)
        
        # Instructions
        instructions = QLabel(
            "<b>Joint Angles (Real-time)</b><br>"
            "Displays absolute and relative angles for each joint."
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        
        # Table for angles
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Joint", "Absolute (°)", "Relative (°)"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        layout.addWidget(self.table)
        
        # Close button
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        layout.addWidget(close_btn)
    
    def update_angles(self):
        """Update the angle display"""
        if not self.parent_widget:
            return
        
        chain = self.parent_widget.chain
        self.table.setRowCount(chain.num_joints)
        
        for i in range(chain.num_joints):
            # Joint label
            joint_item = QTableWidgetItem(f"Joint {i}")
            joint_item.setFlags(joint_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(i, 0, joint_item)
            
            if i == 0:
                # Base joint - show absolute angle
                if chain.num_joints > 1:
                    direction = chain.joints[1] - chain.joints[0]
                    angle = np.arctan2(direction[1], direction[0])
                    abs_angle_item = QTableWidgetItem(f"{np.degrees(angle):.1f}")
                    abs_angle_item.setFlags(abs_angle_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                    self.table.setItem(i, 1, abs_angle_item)
                    
                    rel_angle_item = QTableWidgetItem("N/A")
                    rel_angle_item.setFlags(rel_angle_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                    self.table.setItem(i, 2, rel_angle_item)
            else:
                # Calculate absolute angle
                direction = chain.joints[i] - chain.joints[i-1]
                angle = np.arctan2(direction[1], direction[0])
                abs_angle_item = QTableWidgetItem(f"{np.degrees(angle):.1f}")
                abs_angle_item.setFlags(abs_angle_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.table.setItem(i, 1, abs_angle_item)
                
                # Calculate relative angle to previous link
                if i > 1:
                    prev_direction = chain.joints[i-1] - chain.joints[i-2]
                    prev_angle = np.arctan2(prev_direction[1], prev_direction[0])
                    relative_angle = angle - prev_angle
                    # Normalize to [-180, 180]
                    relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
                    rel_angle_item = QTableWidgetItem(f"{np.degrees(relative_angle):.1f}")
                else:
                    rel_angle_item = QTableWidgetItem("N/A")
                
                rel_angle_item.setFlags(rel_angle_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.table.setItem(i, 2, rel_angle_item)


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
        
        # Initialize joints in a straight line
        self.joints = np.zeros((num_joints, 2))
        self.joints[0] = self.base_position
        for i in range(1, num_joints):
            self.joints[i] = self.joints[i-1] + np.array([link_length, 0])
        
        self.total_length = sum(self.link_lengths)
        self.tolerance = 0.5
        self.max_iterations = 10
        self.current_iterations = 0  # Track actual iterations used
        
        # Snap smoothing
        self.previous_joints = self.joints.copy()
        self.target_joints = self.joints.copy()
        self.is_interpolating = False
        self.interpolation_progress = 0.0
        self.interpolation_speed = 0.02  # How fast to interpolate (0-1) - slower for smoother animation
        self.snap_threshold = 30.0  # Distance threshold to detect snap
        
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
    
    def detect_snap(self, new_joints):
        """
        Detect if the new solution represents a snap
        
        Args:
            new_joints: New joint positions from FABRIK
            
        Returns:
            bool: True if snap detected
        """
        # Calculate total movement of all joints
        total_movement = 0
        for i in range(1, self.num_joints):  # Skip base
            movement = np.linalg.norm(new_joints[i] - self.joints[i])
            total_movement += movement
        
        avg_movement = total_movement / (self.num_joints - 1)
        return avg_movement > self.snap_threshold
    
    def generate_collision_sequence(self, collision_joint_index):
        """
        Generate the sequence of joints to reverse when collision occurs
        Pattern: rC -> rB -> rC -> rB -> rA -> rC -> rB -> rC -> rB -> rA -> ...
        
        Args:
            collision_joint_index: Index of joint that caused collision (0-based link index)
            
        Returns:
            List of joint indices in reversal order
        """
        sequence = []
        
        # Start from the colliding joint and work backwards
        # Pattern: collision_joint, prev_joint, collision_joint, prev_joint, prev_prev_joint, ...
        joints_involved = list(range(collision_joint_index + 1))  # All joints up to collision point
        
        if len(joints_involved) == 0:
            return sequence
        
        # Build pattern
        pattern_length = len(joints_involved) * 3  # Repeat pattern 3 times for good measure
        for i in range(pattern_length):
            # Determine which joint to add based on pattern
            if i == 0:
                # First is always the collision joint
                sequence.append(collision_joint_index)
            else:
                # Calculate position in pattern
                pattern_pos = i % (collision_joint_index + 2)
                if pattern_pos <= collision_joint_index:
                    joint_idx = collision_joint_index - pattern_pos
                    sequence.append(joint_idx)
        
        return sequence
    
    def check_interpolation_collision(self, joints):
        """
        Check if current interpolated pose has any collisions
        
        Args:
            joints: Current joint positions
            
        Returns:
            Tuple of (has_collision, furthest_collision_joint_index)
        """
        if not self.obstacles:
            return False, -1
        
        furthest_collision = -1
        
        # Check joint collisions
        for i in range(1, len(joints)):
            for obstacle in self.obstacles:
                if obstacle.is_point_inside(joints[i]):
                    furthest_collision = max(furthest_collision, i - 1)  # Link index
        
        # Check link collisions
        for i in range(len(joints) - 1):
            for obstacle in self.obstacles:
                tangent = obstacle.get_tangent_direction(joints[i], joints[i + 1])
                if tangent is not None:
                    furthest_collision = max(furthest_collision, i)
        
        return furthest_collision >= 0, furthest_collision
    
    def update_interpolation(self):
        """
        Update the interpolation between poses with collision-aware angle reversal
        Should be called each frame when is_interpolating is True
        """
        if not self.is_interpolating:
            return
        
        # Increment progress
        self.interpolation_progress += self.interpolation_speed
        
        if self.interpolation_progress >= 1.0:
            # Interpolation complete
            self.joints = self.target_joints.copy()
            self.is_interpolating = False
            self.interpolation_progress = 0.0
            self.angle_reversals.clear()
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
            
            # Apply reversals if any joints need to go the other way
            for joint_idx, reversed in self.angle_reversals.items():
                if reversed and joint_idx < len(angle_diffs):
                    # Reverse direction: instead of going angle_diff, go (2π - angle_diff) in opposite direction
                    # This means: go the long way around
                    angle_diffs[joint_idx] = angle_diffs[joint_idx] - 2 * np.pi * np.sign(angle_diffs[joint_idx])
            
            # Interpolate angles
            current_angles = start_angles + angle_diffs * self.interpolation_progress
            
            # Reconstruct joints from interpolated angles
            temp_joints = self.set_joints_from_angles(current_angles, self.base_position)
            
            # Check for collisions
            has_collision, collision_joint_idx = self.check_interpolation_collision(temp_joints)
            
            if has_collision:
                # Generate or update collision sequence if collision joint changed
                if collision_joint_idx != self.last_collision_joint:
                    self.last_collision_joint = collision_joint_idx
                    self.collision_sequence_index = 0
                    self.collision_sequence = self.generate_collision_sequence(collision_joint_idx)
                
                # Apply next reversal in sequence
                if len(self.collision_sequence) > 0 and self.collision_sequence_index < len(self.collision_sequence):
                    joint_to_reverse = self.collision_sequence[self.collision_sequence_index]
                    
                    # Toggle reversal for this joint
                    if joint_to_reverse in self.angle_reversals:
                        self.angle_reversals[joint_to_reverse] = not self.angle_reversals[joint_to_reverse]
                    else:
                        self.angle_reversals[joint_to_reverse] = True
                    
                    # Move to next in sequence
                    self.collision_sequence_index += 1
                    
                    # Recalculate with new reversals
                    angle_diffs = target_angles - start_angles
                    angle_diffs = np.arctan2(np.sin(angle_diffs), np.cos(angle_diffs))
                    
                    for joint_idx, reversed in self.angle_reversals.items():
                        if reversed and joint_idx < len(angle_diffs):
                            angle_diffs[joint_idx] = angle_diffs[joint_idx] - 2 * np.pi * np.sign(angle_diffs[joint_idx])
                    
                    current_angles = start_angles + angle_diffs * self.interpolation_progress
                    temp_joints = self.set_joints_from_angles(current_angles, self.base_position)
            
            # Apply the interpolated joints
            self.joints = temp_joints
            
            # Final collision push-out (as backup)
            if self.obstacles:
                for i in range(1, self.num_joints):
                    for obstacle in self.obstacles:
                        if obstacle.is_point_inside(self.joints[i]):
                            self.joints[i] = obstacle.get_nearest_boundary_point(self.joints[i])
    
    def solve(self, target_position):
        """
        Solve FABRIK IK for target position with obstacle avoidance
        
        Args:
            target_position: (x, y) tuple for target
            
        Returns:
            bool: True if converged within tolerance, False if max iterations reached
        """
        target = np.array(target_position, dtype=float)
        
        # If currently interpolating, don't solve - just update interpolation
        if self.is_interpolating:
            self.update_interpolation()
            return True
        
        # Store current state before solving
        old_joints = self.joints.copy()
        
        # Check if target is reachable
        distance = np.linalg.norm(target - self.base_position)
        
        if distance > self.total_length:
            # Target is unreachable - stretch toward it
            direction = (target - self.base_position) / distance
            new_joints = np.zeros_like(self.joints)
            new_joints[0] = self.base_position
            for i in range(1, self.num_joints):
                new_joints[i] = new_joints[i-1] + direction * self.link_lengths[i-1]
            
            # Check for snap
            if self.detect_snap(new_joints):
                self.previous_joints = old_joints
                self.target_joints = new_joints
                self.is_interpolating = True
                self.interpolation_progress = 0.0
                self.angle_reversals.clear()
                self.collision_sequence_index = 0
                self.last_collision_joint = -1
            else:
                self.joints = new_joints
                
            self.current_iterations = 0
            return False
        
        # Target is reachable - run FABRIK with obstacle avoidance
        # Use a temporary joints array to compute the solution
        temp_joints = self.joints.copy()
        iterations = 0
        diff = np.linalg.norm(temp_joints[-1] - target)
        
        while diff > self.tolerance and iterations < self.max_iterations:
            # Forward reaching - start from end effector
            temp_joints[-1] = target
            
            for i in range(self.num_joints - 2, -1, -1):
                # Calculate desired next position
                direction = temp_joints[i] - temp_joints[i + 1]
                distance = np.linalg.norm(direction)
                if distance > 0:
                    direction = direction / distance
                
                # Check for link collision with obstacles
                new_pos = temp_joints[i + 1] + direction * self.link_lengths[i]
                
                if self.obstacles:
                    for obstacle in self.obstacles:
                        # Check if link would pass through obstacle
                        tangent_dir = obstacle.get_tangent_direction(temp_joints[i + 1], new_pos)
                        if tangent_dir is not None:
                            # Link hits obstacle, redirect along tangent
                            direction = tangent_dir
                            new_pos = temp_joints[i + 1] + direction * self.link_lengths[i]
                
                temp_joints[i] = new_pos
                
                # Check if joint ended up inside obstacle, push to boundary
                if self.obstacles:
                    for obstacle in self.obstacles:
                        if obstacle.is_point_inside(temp_joints[i]):
                            temp_joints[i] = obstacle.get_nearest_boundary_point(temp_joints[i])
            
            # Backward reaching - start from base
            temp_joints[0] = self.base_position
            
            for i in range(self.num_joints - 1):
                # Calculate desired next position
                direction = temp_joints[i + 1] - temp_joints[i]
                distance = np.linalg.norm(direction)
                if distance > 0:
                    direction = direction / distance
                
                # Check for link collision with obstacles
                new_pos = temp_joints[i] + direction * self.link_lengths[i]
                
                if self.obstacles:
                    for obstacle in self.obstacles:
                        # Check if link would pass through obstacle
                        tangent_dir = obstacle.get_tangent_direction(temp_joints[i], new_pos)
                        if tangent_dir is not None:
                            # Link hits obstacle, redirect along tangent
                            direction = tangent_dir
                            new_pos = temp_joints[i] + direction * self.link_lengths[i]
                
                temp_joints[i + 1] = new_pos
                
                # Check if joint ended up inside obstacle, push to boundary
                if self.obstacles:
                    for obstacle in self.obstacles:
                        if obstacle.is_point_inside(temp_joints[i + 1]):
                            temp_joints[i + 1] = obstacle.get_nearest_boundary_point(temp_joints[i + 1])
            
            diff = np.linalg.norm(temp_joints[-1] - target)
            iterations += 1
        
        self.current_iterations = iterations
        
        # Check if solution represents a snap
        if self.detect_snap(temp_joints):
            # Start smooth interpolation
            self.previous_joints = old_joints
            self.target_joints = temp_joints
            self.is_interpolating = True
            self.interpolation_progress = 0.0
            self.angle_reversals.clear()
            self.collision_sequence_index = 0
            self.last_collision_joint = -1
        else:
            # No snap, apply directly
            self.joints = temp_joints
        
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


class FABRIKWidget(QMainWindow):
    """Main application window for FABRIK simulator"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FABRIK Interactive Kinematic Chain Simulator")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize obstacles
        self.obstacles = []
        
        # Initialize FABRIK chain
        self.chain = FABRIKChain(base_position=(100, 300), num_joints=5, link_length=60, obstacles=self.obstacles)
        self.target_position = np.array([400, 300])
        
        # Interaction state
        self.dragging_target = False
        self.dragging_base = False
        self.dragging_joint = None
        self.dragging_obstacle = None
        self.show_reach_circle = True
        self.auto_solve = True
        self.manual_mode = False  # Flag to disable auto-solve during manual positioning
        
        # Obstacle mode
        self.obstacle_mode = False
        
        # End effector selection (default to last joint)
        self.end_effector_index = len(self.chain.joints) - 1
        
        # Path drawing
        self.drawing_path = False
        self.path_points = []
        self.raw_path_points = []  # Store raw clicked points
        self.current_path_index = 0
        self.follow_path = False
        self.path_speed = 2.0
        self.path_step_size = 10.0  # Distance between interpolated points
        
        self.setup_ui()
        
        # Timer for smooth updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(16)  # ~60 FPS
    
    def setup_ui(self):
        """Setup the user interface"""
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Left side - Plot area
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.setRange(xRange=[0, 600], yRange=[0, 600])
        self.plot_widget.setLabel('left', 'Y')
        self.plot_widget.setLabel('bottom', 'X')
        self.plot_widget.setTitle('FABRIK Inverse Kinematics')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        
        # Enable default mouse interaction
        # Left click = pan, right click = zoom box, wheel = zoom
        self.plot_widget.getViewBox().setMenuEnabled(False)  # Disable context menu
        
        # Enable mouse interaction for our custom handling
        self.plot_widget.scene().sigMouseClicked.connect(self.on_mouse_click)
        self.plot_widget.scene().sigMouseMoved.connect(self.on_mouse_move)
        
        # Store mouse press state
        self.mouse_pressed = False
        
        # Create plot items
        self.links_plot = pg.PlotDataItem(pen=pg.mkPen(color='b', width=3))
        self.joints_plot = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('r'))
        self.end_effector_plot = pg.ScatterPlotItem(size=18, brush=pg.mkBrush('magenta'), symbol='star')
        self.base_plot = pg.ScatterPlotItem(size=15, brush=pg.mkBrush('g'), symbol='s')
        self.target_plot = pg.ScatterPlotItem(size=15, brush=pg.mkBrush('orange'), symbol='t')
        self.reach_circle = pg.PlotDataItem(pen=pg.mkPen(color='g', width=1, style=Qt.PenStyle.DashLine))
        self.path_plot = pg.PlotDataItem(pen=pg.mkPen(color='cyan', width=2, style=Qt.PenStyle.DotLine))
        self.path_points_plot = pg.ScatterPlotItem(size=6, brush=pg.mkBrush('cyan', alpha=128), symbol='o')
        self.raw_path_points_plot = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('yellow'), symbol='o')
        
        # Obstacle visualization items (will be updated dynamically)
        self.obstacle_plots = []  # List of (center_plot, circle_plot) tuples
        
        self.plot_widget.addItem(self.links_plot)
        self.plot_widget.addItem(self.joints_plot)
        self.plot_widget.addItem(self.end_effector_plot)
        self.plot_widget.addItem(self.base_plot)
        self.plot_widget.addItem(self.target_plot)
        self.plot_widget.addItem(self.reach_circle)
        self.plot_widget.addItem(self.path_plot)
        self.plot_widget.addItem(self.path_points_plot)
        self.plot_widget.addItem(self.raw_path_points_plot)
        
        main_layout.addWidget(self.plot_widget, stretch=3)
        
        # Right side - Control panel in scroll area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        
        control_panel = self.create_control_panel()
        scroll_area.setWidget(control_panel)
        
        main_layout.addWidget(scroll_area, stretch=1)
        
        self.update_plot()
    
    def create_control_panel(self):
        """Create the control panel widget"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Chain configuration group
        chain_group = QGroupBox("Chain Configuration")
        chain_layout = QVBoxLayout()
        
        # Number of joints
        joints_layout = QHBoxLayout()
        joints_layout.addWidget(QLabel("Number of Joints:"))
        self.joints_spinbox = QSpinBox()
        self.joints_spinbox.setMinimum(2)
        self.joints_spinbox.setMaximum(20)
        self.joints_spinbox.setValue(self.chain.num_joints)
        self.joints_spinbox.valueChanged.connect(self.on_joints_changed)
        joints_layout.addWidget(self.joints_spinbox)
        chain_layout.addLayout(joints_layout)
        
        # Add/Remove joint buttons
        buttons_layout = QHBoxLayout()
        add_joint_btn = QPushButton("Add Joint")
        add_joint_btn.clicked.connect(self.add_joint)
        remove_joint_btn = QPushButton("Remove Joint")
        remove_joint_btn.clicked.connect(self.remove_joint)
        buttons_layout.addWidget(add_joint_btn)
        buttons_layout.addWidget(remove_joint_btn)
        chain_layout.addLayout(buttons_layout)
        
        # Configure joints button
        config_joints_btn = QPushButton("Configure Joint Positions")
        config_joints_btn.clicked.connect(self.open_joint_config)
        chain_layout.addWidget(config_joints_btn)
        
        # Show angles button
        show_angles_btn = QPushButton("Show Joint Angles")
        show_angles_btn.clicked.connect(self.open_angles_monitor)
        chain_layout.addWidget(show_angles_btn)
        
        # Link length
        length_layout = QHBoxLayout()
        length_layout.addWidget(QLabel("Default Link Length:"))
        self.link_length_spinbox = QDoubleSpinBox()
        self.link_length_spinbox.setMinimum(10)
        self.link_length_spinbox.setMaximum(200)
        self.link_length_spinbox.setValue(60)
        self.link_length_spinbox.setSingleStep(5)
        length_layout.addWidget(self.link_length_spinbox)
        chain_layout.addLayout(length_layout)
        
        chain_group.setLayout(chain_layout)
        layout.addWidget(chain_group)
        
        # Target configuration group
        target_group = QGroupBox("Target Configuration")
        target_layout = QVBoxLayout()
        
        # Target X
        target_x_layout = QHBoxLayout()
        target_x_layout.addWidget(QLabel("Target X:"))
        self.target_x_spinbox = QDoubleSpinBox()
        self.target_x_spinbox.setMinimum(0)
        self.target_x_spinbox.setMaximum(600)
        self.target_x_spinbox.setValue(self.target_position[0])
        self.target_x_spinbox.valueChanged.connect(self.on_target_changed)
        target_x_layout.addWidget(self.target_x_spinbox)
        target_layout.addLayout(target_x_layout)
        
        # Target Y
        target_y_layout = QHBoxLayout()
        target_y_layout.addWidget(QLabel("Target Y:"))
        self.target_y_spinbox = QDoubleSpinBox()
        self.target_y_spinbox.setMinimum(0)
        self.target_y_spinbox.setMaximum(600)
        self.target_y_spinbox.setValue(self.target_position[1])
        self.target_y_spinbox.valueChanged.connect(self.on_target_changed)
        target_y_layout.addWidget(self.target_y_spinbox)
        target_layout.addLayout(target_y_layout)
        
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)
        
        # Display options group
        display_group = QGroupBox("Display Options")
        display_layout = QVBoxLayout()
        
        self.show_reach_checkbox = QCheckBox("Show Reach Circle")
        self.show_reach_checkbox.setChecked(True)
        self.show_reach_checkbox.stateChanged.connect(self.on_display_changed)
        display_layout.addWidget(self.show_reach_checkbox)
        
        self.auto_solve_checkbox = QCheckBox("Auto Solve")
        self.auto_solve_checkbox.setChecked(True)
        self.auto_solve_checkbox.stateChanged.connect(self.on_auto_solve_changed)
        display_layout.addWidget(self.auto_solve_checkbox)
        
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        # End Effector Selection
        effector_group = QGroupBox("End Effector Selection")
        effector_layout = QVBoxLayout()
        
        effector_info_layout = QHBoxLayout()
        effector_info_layout.addWidget(QLabel("End Effector Joint:"))
        self.effector_spinbox = QSpinBox()
        self.effector_spinbox.setMinimum(0)
        self.effector_spinbox.setMaximum(self.chain.num_joints - 1)
        self.effector_spinbox.setValue(self.end_effector_index)
        self.effector_spinbox.valueChanged.connect(self.on_effector_changed)
        effector_info_layout.addWidget(self.effector_spinbox)
        effector_layout.addLayout(effector_info_layout)
        
        effector_note = QLabel("0 = Base, " + str(self.chain.num_joints - 1) + " = Tip")
        effector_note.setStyleSheet("font-size: 9pt; color: gray;")
        effector_layout.addWidget(effector_note)
        self.effector_note_label = effector_note
        
        effector_group.setLayout(effector_layout)
        layout.addWidget(effector_group)
        
        # Path Drawing Controls
        path_group = QGroupBox("Path Drawing")
        path_layout = QVBoxLayout()
        
        path_buttons_layout = QHBoxLayout()
        self.draw_path_btn = QPushButton("Start Drawing")
        self.draw_path_btn.setCheckable(True)
        self.draw_path_btn.clicked.connect(self.toggle_path_drawing)
        path_buttons_layout.addWidget(self.draw_path_btn)
        
        clear_path_btn = QPushButton("Clear Path")
        clear_path_btn.clicked.connect(self.clear_path)
        path_buttons_layout.addWidget(clear_path_btn)
        path_layout.addLayout(path_buttons_layout)
        
        self.follow_path_btn = QPushButton("Follow Path")
        self.follow_path_btn.setCheckable(True)
        self.follow_path_btn.clicked.connect(self.toggle_follow_path)
        self.follow_path_btn.setEnabled(False)
        path_layout.addWidget(self.follow_path_btn)
        
        # Path speed control
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Path Speed:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(10)
        self.speed_slider.setValue(2)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        speed_layout.addWidget(self.speed_slider)
        self.speed_label = QLabel("2.0")
        speed_layout.addWidget(self.speed_label)
        path_layout.addLayout(speed_layout)
        
        # Path smoothness control
        smooth_layout = QHBoxLayout()
        smooth_layout.addWidget(QLabel("Path Step Size:"))
        self.step_size_slider = QSlider(Qt.Orientation.Horizontal)
        self.step_size_slider.setMinimum(5)
        self.step_size_slider.setMaximum(50)
        self.step_size_slider.setValue(10)
        self.step_size_slider.valueChanged.connect(self.on_step_size_changed)
        smooth_layout.addWidget(self.step_size_slider)
        self.step_size_label = QLabel("10")
        smooth_layout.addWidget(self.step_size_label)
        path_layout.addLayout(smooth_layout)
        
        path_group.setLayout(path_layout)
        layout.addWidget(path_group)
        
        # Solver controls
        solver_group = QGroupBox("Solver Controls")
        solver_layout = QVBoxLayout()
        
        solve_btn = QPushButton("Solve Once")
        solve_btn.clicked.connect(self.solve_once)
        solver_layout.addWidget(solve_btn)
        
        reset_btn = QPushButton("Reset Chain")
        reset_btn.clicked.connect(self.reset_chain)
        solver_layout.addWidget(reset_btn)
        
        # Snap smoothing controls
        snap_threshold_layout = QHBoxLayout()
        snap_threshold_layout.addWidget(QLabel("Snap Threshold:"))
        self.snap_threshold_spin = QDoubleSpinBox()
        self.snap_threshold_spin.setMinimum(5.0)
        self.snap_threshold_spin.setMaximum(100.0)
        self.snap_threshold_spin.setValue(30.0)
        self.snap_threshold_spin.setSingleStep(5.0)
        self.snap_threshold_spin.valueChanged.connect(self.on_snap_threshold_changed)
        snap_threshold_layout.addWidget(self.snap_threshold_spin)
        solver_layout.addLayout(snap_threshold_layout)
        
        interp_speed_layout = QHBoxLayout()
        interp_speed_layout.addWidget(QLabel("Smooth Speed:"))
        self.interp_speed_spin = QDoubleSpinBox()
        self.interp_speed_spin.setMinimum(0.01)
        self.interp_speed_spin.setMaximum(1.0)
        self.interp_speed_spin.setValue(0.02)
        self.interp_speed_spin.setSingleStep(0.01)
        self.interp_speed_spin.valueChanged.connect(self.on_interp_speed_changed)
        interp_speed_layout.addWidget(self.interp_speed_spin)
        solver_layout.addLayout(interp_speed_layout)
        
        # Smoothing status
        self.smoothing_status_label = QLabel("Interpolating: No")
        self.smoothing_status_label.setStyleSheet("font-size: 9pt; color: gray;")
        solver_layout.addWidget(self.smoothing_status_label)
        
        solver_group.setLayout(solver_layout)
        layout.addWidget(solver_group)
        
        # Obstacle Controls
        obstacle_group = QGroupBox("Obstacle Avoidance")
        obstacle_layout = QVBoxLayout()
        
        # Add/Remove obstacle buttons
        obstacle_buttons = QHBoxLayout()
        self.add_obstacle_btn = QPushButton("Add Obstacle")
        self.add_obstacle_btn.setCheckable(True)
        self.add_obstacle_btn.clicked.connect(self.toggle_obstacle_mode)
        obstacle_buttons.addWidget(self.add_obstacle_btn)
        
        self.clear_obstacles_btn = QPushButton("Clear All")
        self.clear_obstacles_btn.clicked.connect(self.clear_obstacles)
        obstacle_buttons.addWidget(self.clear_obstacles_btn)
        obstacle_layout.addLayout(obstacle_buttons)
        
        # Obstacle radius control
        radius_layout = QHBoxLayout()
        radius_layout.addWidget(QLabel("Warning Radius:"))
        self.obstacle_radius_spin = QDoubleSpinBox()
        self.obstacle_radius_spin.setMinimum(10.0)
        self.obstacle_radius_spin.setMaximum(200.0)
        self.obstacle_radius_spin.setValue(50.0)
        self.obstacle_radius_spin.setSingleStep(5.0)
        radius_layout.addWidget(self.obstacle_radius_spin)
        obstacle_layout.addLayout(radius_layout)
        
        # Obstacle count display
        self.obstacle_count_label = QLabel("Obstacles: 0")
        obstacle_layout.addWidget(self.obstacle_count_label)
        
        obstacle_note = QLabel("Click 'Add Obstacle' then click on plot to place obstacles. Ctrl+drag to move them.")
        obstacle_note.setWordWrap(True)
        obstacle_note.setStyleSheet("font-size: 9pt; color: gray;")
        obstacle_layout.addWidget(obstacle_note)
        
        obstacle_group.setLayout(obstacle_layout)
        layout.addWidget(obstacle_group)
        
        # Info display
        info_group = QGroupBox("Information")
        info_layout = QVBoxLayout()
        
        self.info_label = QLabel()
        self.info_label.setWordWrap(True)
        info_layout.addWidget(self.info_label)
        
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        # Instructions
        instructions = QLabel(
            "<b>Controls:</b><br>"
            "• <b>Hold Ctrl + Left-drag:</b> Drag objects<br>"
            "• <b>Left-drag:</b> Pan the view<br>"
            "• <b>Right-drag:</b> Zoom box<br>"
            "• <b>Scroll:</b> Zoom in/out<br>"
            "• Path drawing restricted to reach circle<br>"
            "• Select end effector to change reach radius"
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        
        layout.addStretch()
        return panel
    
    def update_plot(self):
        """Update the plot with current chain state"""
        # Follow path if enabled
        if self.follow_path and len(self.path_points) > 0:
            target = self.path_points[self.current_path_index]
            self.target_position = np.array(target)
            
            # Solve for current waypoint
            if self.auto_solve and not self.manual_mode:
                converged = self.solve_for_end_effector()
                
                # Check if close enough to current waypoint OR if max iterations reached without convergence
                distance_to_waypoint = np.linalg.norm(
                    self.chain.joints[self.end_effector_index] - self.target_position
                )
                if distance_to_waypoint < 5.0 or not converged:
                    # Move to next waypoint
                    self.current_path_index = (self.current_path_index + 1) % len(self.path_points)
        else:
            # Only auto-solve if not in manual mode and not following path
            if self.auto_solve and not self.manual_mode:
                self.solve_for_end_effector()
        
        # Update links
        self.links_plot.setData(self.chain.joints[:, 0], self.chain.joints[:, 1])
        
        # Update joints (exclude end effector as it's shown separately)
        non_effector_joints = np.delete(self.chain.joints, self.end_effector_index, axis=0)
        if len(non_effector_joints) > 0:
            self.joints_plot.setData(non_effector_joints[:, 0], non_effector_joints[:, 1])
        else:
            self.joints_plot.clear()
        
        # Update end effector (highlighted)
        effector_pos = self.chain.joints[self.end_effector_index]
        self.end_effector_plot.setData([effector_pos[0]], [effector_pos[1]])
        
        # Update base
        self.base_plot.setData([self.chain.base_position[0]], [self.chain.base_position[1]])
        
        # Update target
        self.target_plot.setData([self.target_position[0]], [self.target_position[1]])
        
        # Calculate reach circle radius (distance from base to end effector)
        reach_radius = sum(self.chain.link_lengths[:self.end_effector_index]) if self.end_effector_index > 0 else 0
        
        # Update reach circle
        if self.show_reach_circle:
            theta = np.linspace(0, 2 * np.pi, 100)
            circle_x = self.chain.base_position[0] + reach_radius * np.cos(theta)
            circle_y = self.chain.base_position[1] + reach_radius * np.sin(theta)
            self.reach_circle.setData(circle_x, circle_y)
        else:
            self.reach_circle.clear()
        
        # Update path
        if len(self.path_points) > 0:
            path_array = np.array(self.path_points)
            self.path_plot.setData(path_array[:, 0], path_array[:, 1])
            self.path_points_plot.setData(path_array[:, 0], path_array[:, 1])
        else:
            self.path_plot.clear()
            self.path_points_plot.clear()
        
        # Update raw path points (yellow markers for clicked points)
        if len(self.raw_path_points) > 0:
            raw_array = np.array(self.raw_path_points)
            self.raw_path_points_plot.setData(raw_array[:, 0], raw_array[:, 1])
        else:
            self.raw_path_points_plot.clear()
        
        # Update info
        distance = np.linalg.norm(self.target_position - self.chain.base_position)
        reachable = distance <= reach_radius
        end_effector = self.chain.joints[self.end_effector_index]
        error = np.linalg.norm(end_effector - self.target_position)
        
        info_text = f"""
        <b>Chain Stats:</b><br>
        Joints: {self.chain.num_joints}<br>
        Total Length: {self.chain.total_length:.1f}<br>
        End Effector: Joint {self.end_effector_index}<br>
        Reach Radius: {reach_radius:.1f}<br>
        <br>
        <b>Target Info:</b><br>
        Position: ({self.target_position[0]:.1f}, {self.target_position[1]:.1f})<br>
        Distance: {distance:.1f}<br>
        Reachable: {'Yes' if reachable else 'No'}<br>
        Error: {error:.2f}<br>
        <br>
        <b>Solver Info:</b><br>
        Max Iterations: {self.chain.max_iterations}<br>
        Last Iterations: {self.chain.current_iterations}<br>
        Tolerance: {self.chain.tolerance}<br>
        Interpolating: {'Yes' if self.chain.is_interpolating else 'No'}<br>
        {f'Progress: {self.chain.interpolation_progress*100:.0f}%' if self.chain.is_interpolating else ''}<br>
        <br>
        <b>Path Info:</b><br>
        Points: {len(self.path_points)}<br>
        Following: {'Yes' if self.follow_path else 'No'}
        """
        self.info_label.setText(info_text.strip())
        
        # Update smoothing status label
        if self.chain.is_interpolating:
            reversals_count = len([v for v in self.chain.angle_reversals.values() if v])
            status_text = f"Interpolating: Yes ({self.chain.interpolation_progress*100:.0f}%)"
            if reversals_count > 0:
                status_text += f" | Reversals: {reversals_count}"
            self.smoothing_status_label.setText(status_text)
            self.smoothing_status_label.setStyleSheet("font-size: 9pt; color: green;")
        else:
            self.smoothing_status_label.setText("Interpolating: No")
            self.smoothing_status_label.setStyleSheet("font-size: 9pt; color: gray;")
        
        # Update obstacles visualization
        self.update_obstacle_plots()
        
        # Update obstacle count label
        self.obstacle_count_label.setText(f"Obstacles: {len(self.obstacles)}")
    
    def update_obstacle_plots(self):
        """Update obstacle visualization on plot"""
        # Remove old plots
        for plots in self.obstacle_plots:
            for plot in plots:
                self.plot_widget.removeItem(plot)
        self.obstacle_plots.clear()
        
        # Add new plots for each obstacle
        for obstacle in self.obstacles:
            # Draw center point
            center_plot = pg.ScatterPlotItem(
                pos=np.array([obstacle.position]),
                size=15,
                brush=pg.mkBrush('red'),
                symbol='x'
            )
            
            # Draw collision circle (solid red)
            angles = np.linspace(0, 2 * np.pi, 50)
            collision_x = obstacle.position[0] + obstacle.collision_radius * np.cos(angles)
            collision_y = obstacle.position[1] + obstacle.collision_radius * np.sin(angles)
            collision_plot = pg.PlotDataItem(
                collision_x, collision_y,
                pen=pg.mkPen(color='red', width=3, style=Qt.PenStyle.SolidLine)
            )
            
            # Draw warning zone circle (dashed orange)
            warning_x = obstacle.position[0] + obstacle.radius * np.cos(angles)
            warning_y = obstacle.position[1] + obstacle.radius * np.sin(angles)
            warning_plot = pg.PlotDataItem(
                warning_x, warning_y,
                pen=pg.mkPen(color='orange', width=2, style=Qt.PenStyle.DashLine)
            )
            
            self.plot_widget.addItem(center_plot)
            self.plot_widget.addItem(collision_plot)
            self.plot_widget.addItem(warning_plot)
            self.obstacle_plots.append((center_plot, collision_plot, warning_plot))
    
    def on_mouse_click(self, event):
        """Handle mouse click events"""
        # Only handle left clicks for object interaction with Ctrl modifier
        if event.button() != Qt.MouseButton.LeftButton:
            return
        
        # Check if Ctrl key is pressed for object dragging
        modifiers = event.modifiers()
        ctrl_pressed = modifiers == Qt.KeyboardModifier.ControlModifier
            
        pos = self.plot_widget.plotItem.vb.mapSceneToView(event.scenePos())
        x, y = pos.x(), pos.y()
        
        # If in obstacle mode, add obstacle on click (no Ctrl needed)
        if self.obstacle_mode:
            radius = self.obstacle_radius_spin.value()
            new_obstacle = Obstacle(position=(x, y), radius=radius)
            self.obstacles.append(new_obstacle)
            event.accept()
            return
        
        # If drawing path, add point only if inside reach circle (no Ctrl needed)
        if self.drawing_path:
            # Calculate reach radius
            reach_radius = sum(self.chain.link_lengths[:self.end_effector_index]) if self.end_effector_index > 0 else 0
            distance_from_base = np.linalg.norm(np.array([x, y]) - self.chain.base_position)
            
            if distance_from_base <= reach_radius:
                self.raw_path_points.append([x, y])
                self.interpolate_path()
                event.accept()
            return
        
        # Only allow dragging with Ctrl key
        if not ctrl_pressed:
            return
        
        # Check if clicking on an obstacle
        for i, obstacle in enumerate(self.obstacles):
            dist = np.linalg.norm(obstacle.position - np.array([x, y]))
            if dist < 20:
                self.dragging_obstacle = i
                self.mouse_pressed = True
                event.accept()
                return
        
        # Check if clicking on target
        target_dist = np.linalg.norm(self.target_position - np.array([x, y]))
        if target_dist < 20:
            self.dragging_target = True
            self.mouse_pressed = True
            event.accept()
            return
        
        # Check if clicking on base
        base_dist = np.linalg.norm(self.chain.base_position - np.array([x, y]))
        if base_dist < 20:
            self.dragging_base = True
            self.mouse_pressed = True
            event.accept()
            return
        
        # Check if clicking on any joint to drag it
        for i, joint in enumerate(self.chain.joints):
            joint_dist = np.linalg.norm(joint - np.array([x, y]))
            if joint_dist < 20:
                self.dragging_joint = i
                self.end_effector_index = i
                self.mouse_pressed = True
                self.effector_spinbox.blockSignals(True)
                self.effector_spinbox.setValue(i)
                self.effector_spinbox.blockSignals(False)
                event.accept()
                return
    
    def on_mouse_move(self, pos):
        """Handle mouse move events"""
        # Only process moves if we're dragging something
        if not (self.dragging_target or self.dragging_base or self.dragging_joint is not None or self.dragging_obstacle is not None):
            return
            
        view_pos = self.plot_widget.plotItem.vb.mapSceneToView(pos)
        x, y = view_pos.x(), view_pos.y()
        
        if self.dragging_obstacle is not None:
            self.obstacles[self.dragging_obstacle].position = np.array([x, y])
        
        if self.dragging_target:
            self.target_position = np.array([x, y])
            self.target_x_spinbox.blockSignals(True)
            self.target_y_spinbox.blockSignals(True)
            self.target_x_spinbox.setValue(x)
            self.target_y_spinbox.setValue(y)
            self.target_x_spinbox.blockSignals(False)
            self.target_y_spinbox.blockSignals(False)
        
        if self.dragging_base:
            self.chain.base_position = np.array([x, y])
            self.chain.joints[0] = self.chain.base_position
        
        if self.dragging_joint is not None:
            # Dragging a joint - set it as target for IK
            self.target_position = np.array([x, y])
            self.target_x_spinbox.blockSignals(True)
            self.target_y_spinbox.blockSignals(True)
            self.target_x_spinbox.setValue(x)
            self.target_y_spinbox.setValue(y)
            self.target_x_spinbox.blockSignals(False)
            self.target_y_spinbox.blockSignals(False)
    
    def mouseReleaseEvent(self, event):
        """Handle mouse release events"""
        self.dragging_target = False
        self.dragging_base = False
        self.dragging_joint = None
        self.dragging_obstacle = None
        self.mouse_pressed = False
    
    def on_joints_changed(self, value):
        """Handle change in number of joints"""
        current = self.chain.num_joints
        if value > current:
            for _ in range(value - current):
                self.add_joint()
        elif value < current:
            for _ in range(current - value):
                self.remove_joint()
        
        # Update end effector spinbox range
        self.effector_spinbox.setMaximum(self.chain.num_joints - 1)
        if self.end_effector_index >= self.chain.num_joints:
            self.end_effector_index = self.chain.num_joints - 1
            self.effector_spinbox.setValue(self.end_effector_index)
        self.effector_note_label.setText("0 = Base, " + str(self.chain.num_joints - 1) + " = Tip")
    
    def add_joint(self):
        """Add a joint to the chain"""
        link_length = self.link_length_spinbox.value()
        self.chain.add_joint(link_length)
        self.joints_spinbox.blockSignals(True)
        self.joints_spinbox.setValue(self.chain.num_joints)
        self.joints_spinbox.blockSignals(False)
        self.effector_spinbox.setMaximum(self.chain.num_joints - 1)
        self.effector_note_label.setText("0 = Base, " + str(self.chain.num_joints - 1) + " = Tip")
    
    def remove_joint(self):
        """Remove a joint from the chain"""
        self.chain.remove_joint()
        self.joints_spinbox.blockSignals(True)
        self.joints_spinbox.setValue(self.chain.num_joints)
        self.joints_spinbox.blockSignals(False)
        self.effector_spinbox.setMaximum(self.chain.num_joints - 1)
        if self.end_effector_index >= self.chain.num_joints:
            self.end_effector_index = self.chain.num_joints - 1
            self.effector_spinbox.setValue(self.end_effector_index)
        self.effector_note_label.setText("0 = Base, " + str(self.chain.num_joints - 1) + " = Tip")
    
    def on_target_changed(self):
        """Handle target position change from spinboxes"""
        self.target_position[0] = self.target_x_spinbox.value()
        self.target_position[1] = self.target_y_spinbox.value()
    
    def on_display_changed(self):
        """Handle display options change"""
        self.show_reach_circle = self.show_reach_checkbox.isChecked()
    
    def on_auto_solve_changed(self):
        """Handle auto-solve toggle"""
        self.auto_solve = self.auto_solve_checkbox.isChecked()
    
    def solve_once(self):
        """Solve FABRIK once"""
        self.chain.solve(self.target_position)
    
    def reset_chain(self):
        """Reset the chain to initial configuration"""
        num_joints = self.chain.num_joints
        link_length = self.link_length_spinbox.value()
        self.chain = FABRIKChain(
            base_position=self.chain.base_position,
            num_joints=num_joints,
            link_length=link_length
        )
    
    def solve_for_end_effector(self):
        """Solve FABRIK to position the selected end effector at target
        
        Returns:
            bool: True if converged, False if max iterations reached
        """
        if self.end_effector_index == self.chain.num_joints - 1:
            # Standard case - last joint is end effector
            return self.chain.solve(self.target_position)
        else:
            # Need to solve for a mid-chain joint
            # Create a sub-chain from base to end effector
            sub_joints = self.chain.joints[:self.end_effector_index + 1].copy()
            sub_lengths = self.chain.link_lengths[:self.end_effector_index]
            
            # Solve for sub-chain (pass obstacles reference)
            temp_chain = FABRIKChain(self.chain.base_position, num_joints=len(sub_joints), obstacles=self.chain.obstacles)
            temp_chain.joints = sub_joints
            temp_chain.link_lengths = sub_lengths
            temp_chain.total_length = sum(sub_lengths) if sub_lengths else 0
            converged = temp_chain.solve(self.target_position)
            
            # Copy iteration count back to main chain
            self.chain.current_iterations = temp_chain.current_iterations
            
            # Update the main chain with solved positions
            self.chain.joints[:self.end_effector_index + 1] = temp_chain.joints
            
            # Update remaining joints to maintain their relative positions
            if self.end_effector_index < self.chain.num_joints - 1:
                for i in range(self.end_effector_index + 1, self.chain.num_joints):
                    direction = self.chain.joints[i] - self.chain.joints[i - 1]
                    distance = np.linalg.norm(direction)
                    if distance > 0:
                        direction = direction / distance
                        self.chain.joints[i] = self.chain.joints[i - 1] + direction * self.chain.link_lengths[i - 1]
            
            return converged
    
    def on_effector_changed(self, value):
        """Handle end effector selection change"""
        self.end_effector_index = value
    
    def open_joint_config(self):
        """Open the joint configuration dialog"""
        # Enter manual mode
        self.manual_mode = True
        
        # Open dialog
        dialog = JointConfigDialog(self.chain, self)
        dialog.exec()
        
        # Exit manual mode
        self.manual_mode = False
    
    def open_angles_monitor(self):
        """Open the joint angles monitor dialog"""
        dialog = JointAnglesDialog(self)
        dialog.exec()
    
    def toggle_path_drawing(self):
        """Toggle path drawing mode"""
        self.drawing_path = self.draw_path_btn.isChecked()
        if self.drawing_path:
            self.draw_path_btn.setText("Stop Drawing")
            self.path_points = []  # Clear existing path
            self.raw_path_points = []  # Clear raw points
            self.follow_path = False
            self.follow_path_btn.setChecked(False)
        else:
            self.draw_path_btn.setText("Start Drawing")
            if len(self.path_points) > 0:
                self.follow_path_btn.setEnabled(True)
    
    def clear_path(self):
        """Clear the drawn path"""
        self.path_points = []
        self.raw_path_points = []
        self.current_path_index = 0
        self.follow_path = False
        self.follow_path_btn.setChecked(False)
        self.follow_path_btn.setEnabled(False)
    
    def toggle_follow_path(self):
        """Toggle path following mode"""
        self.follow_path = self.follow_path_btn.isChecked()
        if self.follow_path:
            self.current_path_index = 0
            self.follow_path_btn.setText("Stop Following")
        else:
            self.follow_path_btn.setText("Follow Path")
    
    def toggle_obstacle_mode(self):
        """Toggle obstacle placement mode"""
        self.obstacle_mode = self.add_obstacle_btn.isChecked()
        if self.obstacle_mode:
            self.add_obstacle_btn.setText("Placing... (click to add)")
            # Disable path drawing while in obstacle mode
            if self.drawing_path:
                self.drawing_path = False
                self.draw_path_btn.setChecked(False)
                self.draw_path_btn.setText("Draw Path")
        else:
            self.add_obstacle_btn.setText("Add Obstacle")
    
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()
    
    def on_speed_changed(self, value):
        """Handle path speed change"""
        self.path_speed = value
        self.speed_label.setText(f"{value}.0")
    
    def on_step_size_changed(self, value):
        """Handle path step size change"""
        self.path_step_size = float(value)
        self.step_size_label.setText(str(value))
        # Re-interpolate existing path with new step size
        if len(self.raw_path_points) > 0:
            self.interpolate_path()
    
    def on_snap_threshold_changed(self, value):
        """Handle snap threshold change"""
        self.chain.snap_threshold = value
    
    def on_interp_speed_changed(self, value):
        """Handle interpolation speed change"""
        self.chain.interpolation_speed = value
    
    def interpolate_path(self):
        """Interpolate path points to create smooth motion"""
        if len(self.raw_path_points) == 0:
            self.path_points = []
            return
        
        if len(self.raw_path_points) == 1:
            self.path_points = [self.raw_path_points[0]]
            return
        
        # Interpolate between consecutive points
        interpolated_points = []
        
        for i in range(len(self.raw_path_points)):
            start_point = np.array(self.raw_path_points[i])
            interpolated_points.append(start_point.tolist())
            
            # Interpolate to next point (or back to first if this is the last point)
            if i < len(self.raw_path_points) - 1:
                end_point = np.array(self.raw_path_points[i + 1])
            else:
                # Close the loop back to first point
                end_point = np.array(self.raw_path_points[0])
            
            # Calculate distance and number of steps
            distance = np.linalg.norm(end_point - start_point)
            if distance > self.path_step_size:
                num_steps = int(distance / self.path_step_size)
                
                for step in range(1, num_steps):
                    t = step / num_steps
                    interp_point = start_point + t * (end_point - start_point)
                    interpolated_points.append(interp_point.tolist())
        
        self.path_points = interpolated_points


def main():
    app = QApplication(sys.argv)
    window = FABRIKWidget()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()