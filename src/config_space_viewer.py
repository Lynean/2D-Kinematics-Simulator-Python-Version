"""
Configuration Space / Joint Space Viewer
Visualizes joint angles and their constraints in configuration space
"""
from PyQt6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton, 
                              QLabel, QCheckBox, QSpinBox)
from PyQt6.QtCore import Qt
import pyqtgraph as pg
import numpy as np


class ConfigSpaceViewer(QDialog):
    """
    Configuration Space Viewer Window
    Shows joint angles plotted in configuration space with constraints
    """
    
    def __init__(self, chain, parent=None):
        super().__init__(parent)
        self.chain = chain
        self.setWindowTitle("Configuration/Joint Space Viewer")
        self.setGeometry(150, 150, 900, 700)
        
        # Tracking settings
        self.enable_tracking = True
        self.max_history = 500
        self.angle_history = []  # List of angle arrays
        
        # C-space obstacle regions
        self.obstacle_scatter_items = []  # ScatterPlotItems for obstacle regions
        
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the user interface"""
        layout = QVBoxLayout(self)
        
        # Control panel
        control_layout = QHBoxLayout()
        
        # Tracking toggle
        self.tracking_checkbox = QCheckBox("Track Path")
        self.tracking_checkbox.setChecked(True)
        self.tracking_checkbox.stateChanged.connect(self.on_tracking_changed)
        control_layout.addWidget(self.tracking_checkbox)
        
        # Clear button
        clear_btn = QPushButton("Clear History")
        clear_btn.clicked.connect(self.clear_history)
        control_layout.addWidget(clear_btn)
        
        # History length control
        control_layout.addWidget(QLabel("Max History:"))
        self.history_spin = QSpinBox()
        self.history_spin.setMinimum(10)
        self.history_spin.setMaximum(2000)
        self.history_spin.setValue(500)
        self.history_spin.setSingleStep(50)
        self.history_spin.valueChanged.connect(self.on_history_length_changed)
        control_layout.addWidget(self.history_spin)
        
        control_layout.addStretch()
        
        # Info label
        self.info_label = QLabel("Configuration Space: Joint angles over time")
        control_layout.addWidget(self.info_label)
        
        layout.addLayout(control_layout)
        
        # Plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setLabel('left', 'Angle (degrees)')
        self.plot_widget.setLabel('bottom', 'Time Step')
        self.plot_widget.setTitle('Joint Configuration Space')
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        
        layout.addWidget(self.plot_widget)
        
        # Create plot items for each joint
        self.joint_plots = []
        self.constraint_regions = []
        colors = ['r', 'g', 'b', 'm', 'c', 'y', 'k', 'orange']
        
        # Initialize plots for current number of joints
        for i in range(self.chain.num_joints - 1):  # Exclude last joint (end effector)
            color = colors[i % len(colors)]
            plot = self.plot_widget.plot([], [], pen=pg.mkPen(color, width=2), 
                                         name=f'Joint {i}')
            self.joint_plots.append(plot)
        
        # Current angle markers
        self.current_markers = pg.ScatterPlotItem(size=10, brush=pg.mkBrush('red'))
        self.plot_widget.addItem(self.current_markers)
        
        # Add constraint visualization in 2D space for first two joints
        self.setup_2d_constraint_view()
        
    def setup_2d_constraint_view(self):
        """Setup 2D constraint space viewer (for first 2 joints)"""
        layout = self.layout()
        
        # Create second plot widget for 2D configuration space
        self.plot_2d = pg.PlotWidget()
        self.plot_2d.setBackground('w')
        self.plot_2d.setLabel('left', 'Joint 1 Angle (degrees, relative to Joint 0)')
        self.plot_2d.setLabel('bottom', 'Joint 0 Angle (degrees, absolute)')
        self.plot_2d.setTitle('2D Configuration Space (Joint 0 vs Joint 1) - Click to set position')
        self.plot_2d.showGrid(x=True, y=True, alpha=0.3)
        self.plot_2d.setAspectLocked(False)
        
        # Enable mouse click events
        self.plot_2d.scene().sigMouseClicked.connect(self.on_2d_plot_clicked)
        
        # Draw constraint boundaries
        if self.chain.num_joints >= 2:
            # Joint 0 limits
            min0, max0 = self.chain.angle_limits[0]
            min0_deg, max0_deg = np.degrees(min0), np.degrees(max0)
            
            # Joint 1 limits (if exists)
            if len(self.chain.angle_limits) > 1:
                min1, max1 = self.chain.angle_limits[1]
                min1_deg, max1_deg = np.degrees(min1), np.degrees(max1)
            else:
                min1_deg, max1_deg = -90, 90
            
            # Draw valid region rectangle
            rect = pg.QtWidgets.QGraphicsRectItem(min0_deg, min1_deg, 
                                                   max0_deg - min0_deg, 
                                                   max1_deg - min1_deg)
            rect.setPen(pg.mkPen('g', width=2))
            rect.setBrush(pg.mkBrush(0, 255, 0, 30))
            self.plot_2d.addItem(rect)
        
        # Path in 2D space
        self.path_2d = self.plot_2d.plot([], [], pen=pg.mkPen('b', width=1), 
                                         symbolBrush='b', symbolSize=3, 
                                         symbolPen=None)
        
        # Current position marker
        self.current_2d = pg.ScatterPlotItem(size=15, brush=pg.mkBrush('red'), 
                                             symbol='o')
        self.plot_2d.addItem(self.current_2d)
        
        layout.addWidget(self.plot_2d)
    
    def update(self):
        """Update the visualization with current joint angles"""
        # Get current angles from cached values (excluding end effector)
        angles = self.chain.joint_angles
        angles_deg = np.degrees(angles)
        
        # Add to history if tracking enabled
        if self.enable_tracking:
            self.angle_history.append(angles_deg.copy())
            
            # Limit history length
            if len(self.angle_history) > self.max_history:
                self.angle_history.pop(0)
        
        # Update time series plots
        if len(self.angle_history) > 0:
            history_array = np.array(self.angle_history)
            time_steps = np.arange(len(self.angle_history))
            
            # Update each joint plot
            for i in range(min(len(self.joint_plots), history_array.shape[1])):
                self.joint_plots[i].setData(time_steps, history_array[:, i])
            
            # Update current position markers
            current_x = [len(self.angle_history) - 1] * len(angles_deg)
            self.current_markers.setData(current_x, angles_deg)
            
            # Update 2D configuration space
            if history_array.shape[1] >= 2:
                self.path_2d.setData(history_array[:, 0], history_array[:, 1])
                self.current_2d.setData([angles_deg[0]], [angles_deg[1]])
        
        # Update info label
        angle_str = ", ".join([f"{a:.1f}°" for a in angles_deg])
        self.info_label.setText(f"Current angles: [{angle_str}] | History: {len(self.angle_history)}")
    
    def clear_history(self):
        """Clear the angle history"""
        self.angle_history.clear()
        
        # Clear plots
        for plot in self.joint_plots:
            plot.setData([], [])
        
        self.current_markers.setData([], [])
        self.path_2d.setData([], [])
        self.current_2d.setData([], [])
        
        self.info_label.setText("History cleared")
    
    def on_tracking_changed(self):
        """Handle tracking toggle"""
        self.enable_tracking = self.tracking_checkbox.isChecked()
        
    def on_history_length_changed(self, value):
        """Handle history length change"""
        self.max_history = value
        
        # Trim history if needed
        if len(self.angle_history) > self.max_history:
            self.angle_history = self.angle_history[-self.max_history:]
    
    def update_joint_count(self):
        """Update plots when joint count changes"""
        # Clear existing plots
        for plot in self.joint_plots:
            self.plot_widget.removeItem(plot)
        
        self.joint_plots.clear()
        
        # Create new plots
        colors = ['r', 'g', 'b', 'm', 'c', 'y', 'k', 'orange']
        for i in range(self.chain.num_joints - 1):
            color = colors[i % len(colors)]
            plot = self.plot_widget.plot([], [], pen=pg.mkPen(color, width=2), 
                                         name=f'Joint {i}')
            self.joint_plots.append(plot)
        
        # Clear history since joint count changed
        self.clear_history()

    def add_obstacle_region(self, cspace_data):
        """
        Add obstacle region to 2D configuration space visualization
        
        Args:
            cspace_data: Dictionary with 'obstacle', 'collision_configs', 'non_collision_configs', 'num_samples'
        """
        collision_configs = cspace_data['collision_configs']
        non_collision_configs = cspace_data['non_collision_configs']
        
        # Extract first two joint angles for 2D visualization
        if self.chain.num_joints >= 2:
            # Visualize non-collision configurations (light transparent grey)
            if len(non_collision_configs) > 0:
                non_collision_array = np.array(non_collision_configs)
                joint0_non_collision = np.degrees(non_collision_array[:, 0])
                joint1_non_collision = np.degrees(non_collision_array[:, 1])
                
                scatter_non_collision = pg.ScatterPlotItem(
                    x=joint0_non_collision,
                    y=joint1_non_collision,
                    size=3,
                    brush=pg.mkBrush(128, 128, 128, 30),  # Light transparent grey
                    pen=None,
                    symbol='o'
                )
                
                self.plot_2d.addItem(scatter_non_collision)
                self.obstacle_scatter_items.append(scatter_non_collision)
            
            # Visualize collision configurations (semi-transparent red)
            if len(collision_configs) > 0:
                collision_array = np.array(collision_configs)
                joint0_collision = np.degrees(collision_array[:, 0])
                joint1_collision = np.degrees(collision_array[:, 1])
                
                scatter_collision = pg.ScatterPlotItem(
                    x=joint0_collision,
                    y=joint1_collision,
                    size=4,
                    brush=pg.mkBrush(255, 0, 0, 100),  # Semi-transparent red
                    pen=None,
                    symbol='o'
                )
                
                self.plot_2d.addItem(scatter_collision)
                self.obstacle_scatter_items.append(scatter_collision)
                
                print(f"Visualized {len(joint0_collision)} collision configs (red) and {len(non_collision_configs)} non-collision configs (grey) in C-space")
            else:
                print(f"Visualized {len(non_collision_configs)} non-collision configs (grey), no collisions found")
    
    def clear_obstacle_regions(self):
        """Clear all obstacle regions from C-space visualization"""
        for scatter in self.obstacle_scatter_items:
            self.plot_2d.removeItem(scatter)
        self.obstacle_scatter_items.clear()
    
    def on_2d_plot_clicked(self, event):
        """Handle mouse click on 2D configuration space plot"""
        if self.chain.num_joints < 2:
            return
        
        # Get mouse position in plot coordinates
        mouse_point = self.plot_2d.plotItem.vb.mapSceneToView(event.scenePos())
        
        # Get clicked angles in degrees
        joint0_deg = mouse_point.x()  # Absolute angle
        joint1_deg = mouse_point.y()  # Relative angle
        
        # Convert to radians
        joint0_rad = np.radians(joint0_deg)  # Absolute
        joint1_rad = np.radians(joint1_deg)  # Relative
        
        # Create angles array with current angles for other joints
        # joint_angles are now relative (first absolute, rest relative)
        new_angles = self.chain.joint_angles.copy()
        new_angles[0] = joint0_rad  # Absolute angle of first link
        if len(new_angles) > 1:
            new_angles[1] = joint1_rad  # Relative angle of second link
        
        # Update chain position using forward kinematics
        new_joints = self.chain.set_joints_from_angles(new_angles, self.chain.base_position)
        self.chain.joints = new_joints
        
        # Update cached angles
        self.chain._update_joint_angles()
        
        # Update visualization
        self.update()
        
        print(f"Set chain to configuration: Joint 0 = {joint0_deg:.1f}° (absolute), Joint 1 = {joint1_deg:.1f}° (relative)")
