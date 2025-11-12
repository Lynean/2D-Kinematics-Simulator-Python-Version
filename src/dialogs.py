
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QPushButton, QLabel, QSpinBox, 
                              QDoubleSpinBox, QGroupBox, QSlider, QCheckBox,
                              QDialog, QTableWidget, QTableWidgetItem, QHeaderView,
                              QScrollArea)
from PyQt6.QtCore import Qt, QTimer
import pyqtgraph as pg
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
            "<b>Configure Joint Positions and Angle Limits</b><br>"
            "Modify X and Y coordinates for each joint.<br>"
            "Set min/max angle limits (in degrees) for servo constraints.<br>"
            "<i>Joint 0 (Base) controls the first link's rotation angle.</i><br>"
            "Link lengths will be maintained when solving."
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        
        # Table for joint positions and angle limits
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels([
            "Joint", "X Position", "Y Position", "Min Angle (°)", "Max Angle (°)"
        ])
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
        """Load current joint positions and angle limits into table"""
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
            
            # Angle limits (convert from radians to degrees)
            min_angle_rad, max_angle_rad = self.chain.angle_limits[i]
            min_angle_deg = np.degrees(min_angle_rad)
            max_angle_deg = np.degrees(max_angle_rad)
            
            min_item = QTableWidgetItem(f"{min_angle_deg:.1f}")
            max_item = QTableWidgetItem(f"{max_angle_deg:.1f}")
            
            # All joints now have editable angle constraints (including base)
            # Base joint (index 0) controls the first link's angle from horizontal
            
            self.table.setItem(i, 3, min_item)
            self.table.setItem(i, 4, max_item)
        
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
            
            # Update angle limits (convert from degrees to radians)
            # Now includes base joint (index 0) which controls first link angle
            for i in range(self.chain.num_joints):
                min_deg = float(self.table.item(i, 3).text())
                max_deg = float(self.table.item(i, 4).text())
                
                # Validate angle range
                if min_deg >= max_deg:
                    raise ValueError(f"Joint {i}: Min angle must be less than max angle")
                
                if max_deg - min_deg > 360:
                    raise ValueError(f"Joint {i}: Angle range cannot exceed 360 degrees")
                
                # Convert to radians and update
                min_rad = np.radians(min_deg)
                max_rad = np.radians(max_deg)
                self.chain.set_joint_limits(i, min_rad, max_rad)
            
            self.info_label.setText("✓ Changes applied successfully!")
            self.info_label.setStyleSheet("color: green; font-style: italic;")
            
        except ValueError as e:
            self.info_label.setText(f"✗ Error: {str(e)}")
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

