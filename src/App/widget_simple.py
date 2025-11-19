from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QPushButton, QLabel, QSpinBox, 
                              QDoubleSpinBox, QGroupBox, QCheckBox, QComboBox,
                              QDialog, QTableWidget, QTableWidgetItem, QHeaderView)
from PyQt6.QtCore import Qt, QTimer
import pyqtgraph as pg
import numpy as np

from src.chain import FABRIKChain
from src.App.config_space_viewer import ConfigSpaceViewer
from src.obstacle import Obstacle
from src.Tools.monte_carlo import MonteCarloSampler
from src.Tools.grid_sampler import GridSampler

class FABRIKWidget(QMainWindow):
    """Simplified FABRIK simulator - Main features only"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FABRIK Kinematic Chain Simulator")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize obstacles
        self.obstacles = []
        self.cspace_obstacles = []
        
        # Initialize FABRIK chain
        self.chain = FABRIKChain(base_position=(100, 300), num_joints=3, link_length=60, obstacles=self.obstacles)
        self.target_position = self.chain.joints[-1].copy()
        
        # Samplers
        self.mc_sampler = MonteCarloSampler(self.chain, num_samples=1000)
        self.grid_sampler = GridSampler(self.chain, grid_resolution=5)
        self.current_sampler = self.mc_sampler  # Default to Monte Carlo
        self.mc_sampler = MonteCarloSampler(self.chain, num_samples=1000)
        
        # Interaction state
        self.show_reach_circle = True
        self.auto_solve = True
        self.obstacle_mode = False
        
        # Config space viewer
        self.config_space_viewer = None
        
        # Mouse interaction
        self.dragging_target = False
        self.dragging_obstacle = None
        
        self.setup_ui()
        
        # Timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)  # ~30 FPS
    
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
        
        # Mouse interaction
        self.plot_widget.scene().sigMouseClicked.connect(self.on_mouse_click)
        self.plot_widget.scene().sigMouseMoved.connect(self.on_mouse_move)
        self.plot_widget.getViewBox().setMenuEnabled(False)
        
        # Create plot items
        self.links_plot = pg.PlotDataItem(pen=pg.mkPen(color='b', width=3))
        self.joints_plot = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('r'))
        self.end_effector_plot = pg.ScatterPlotItem(size=18, brush=pg.mkBrush('magenta'), symbol='star')
        self.base_plot = pg.ScatterPlotItem(size=15, brush=pg.mkBrush('g'), symbol='s')
        self.target_plot = pg.ScatterPlotItem(size=15, brush=pg.mkBrush('orange'), symbol='t')
        self.reach_circle = pg.PlotDataItem(pen=pg.mkPen(color='g', width=1, style=Qt.PenStyle.DashLine))
        
        self.plot_widget.addItem(self.links_plot)
        self.plot_widget.addItem(self.joints_plot)
        self.plot_widget.addItem(self.end_effector_plot)
        self.plot_widget.addItem(self.base_plot)
        self.plot_widget.addItem(self.target_plot)
        self.plot_widget.addItem(self.reach_circle)
        
        # Angle limit visualization items (one arc per joint)
        self.angle_limit_arcs = []
        
        # Obstacle plots
        self.obstacle_plots = []
        
        main_layout.addWidget(self.plot_widget, stretch=3)
        
        # Right side - Control panel
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel)
        
        self.update_plot()
    
    def create_control_panel(self):
        """Create the simplified control panel widget"""
        panel = QWidget()
        panel.setMaximumWidth(350)
        layout = QVBoxLayout(panel)
        
        # Chain configuration
        chain_group = QGroupBox("Chain Configuration")
        chain_layout = QVBoxLayout()
        
        joints_layout = QHBoxLayout()
        joints_layout.addWidget(QLabel("Number of Joints:"))
        self.joints_spinbox = QSpinBox()
        self.joints_spinbox.setMinimum(0)
        self.joints_spinbox.setMaximum(4)
        self.joints_spinbox.setValue(self.chain.num_joints)
        self.joints_spinbox.valueChanged.connect(self.on_joints_changed)
        joints_layout.addWidget(self.joints_spinbox)
        chain_layout.addLayout(joints_layout)
        
        buttons_layout = QHBoxLayout()
        add_joint_btn = QPushButton("Add Joint")
        add_joint_btn.clicked.connect(self.add_joint)
        remove_joint_btn = QPushButton("Remove Joint")
        remove_joint_btn.clicked.connect(self.remove_joint)
        buttons_layout.addWidget(add_joint_btn)
        buttons_layout.addWidget(remove_joint_btn)
        chain_layout.addLayout(buttons_layout)
        
        config_space_btn = QPushButton("Configuration Space Viewer")
        config_space_btn.clicked.connect(self.open_config_space_viewer)
        chain_layout.addWidget(config_space_btn)
        
        # Joint limits configuration button
        joint_limits_btn = QPushButton("Configure Joint Limits")
        joint_limits_btn.clicked.connect(self.open_joint_limits_dialog)
        chain_layout.addWidget(joint_limits_btn)
        
        length_layout = QHBoxLayout()
        length_layout.addWidget(QLabel("Link Length:"))
        self.link_length_spinbox = QDoubleSpinBox()
        self.link_length_spinbox.setMinimum(10)
        self.link_length_spinbox.setMaximum(200)
        self.link_length_spinbox.setValue(60)
        self.link_length_spinbox.setSingleStep(5)
        length_layout.addWidget(self.link_length_spinbox)
        chain_layout.addLayout(length_layout)
        
        chain_group.setLayout(chain_layout)
        layout.addWidget(chain_group)
        
        # Target configuration
        target_group = QGroupBox("Target Position")
        target_layout = QVBoxLayout()
        
        target_x_layout = QHBoxLayout()
        target_x_layout.addWidget(QLabel("X:"))
        self.target_x_spinbox = QDoubleSpinBox()
        self.target_x_spinbox.setMinimum(0)
        self.target_x_spinbox.setMaximum(600)
        self.target_x_spinbox.setValue(self.target_position[0])
        self.target_x_spinbox.valueChanged.connect(self.on_target_changed)
        target_x_layout.addWidget(self.target_x_spinbox)
        target_layout.addLayout(target_x_layout)
        
        target_y_layout = QHBoxLayout()
        target_y_layout.addWidget(QLabel("Y:"))
        self.target_y_spinbox = QDoubleSpinBox()
        self.target_y_spinbox.setMinimum(0)
        self.target_y_spinbox.setMaximum(600)
        self.target_y_spinbox.setValue(self.target_position[1])
        self.target_y_spinbox.valueChanged.connect(self.on_target_changed)
        target_y_layout.addWidget(self.target_y_spinbox)
        target_layout.addLayout(target_y_layout)
        
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)
        
        # Display options
        display_group = QGroupBox("Display Options")
        display_layout = QVBoxLayout()
        
        self.show_reach_checkbox = QCheckBox("Show Reach Circle")
        self.show_reach_checkbox.setChecked(True)
        self.show_reach_checkbox.stateChanged.connect(self.on_display_changed)
        display_layout.addWidget(self.show_reach_checkbox)
        
        self.auto_solve_checkbox = QCheckBox("Auto Solve IK")
        self.auto_solve_checkbox.setChecked(True)
        self.auto_solve_checkbox.stateChanged.connect(self.on_auto_solve_changed)
        display_layout.addWidget(self.auto_solve_checkbox)
        
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        # IK Solver Controls
        solver_group = QGroupBox("IK Solver")
        solver_layout = QVBoxLayout()
        
        # IK Method selection
        ik_method_layout = QHBoxLayout()
        ik_method_layout.addWidget(QLabel("Method:"))
        self.ik_method_combo = QComboBox()
        self.ik_method_combo.addItems(["FABRIK", "CCD", "ASTAR"])
        self.ik_method_combo.setCurrentText("FABRIK")
        self.ik_method_combo.setToolTip("FABRIK: Forward And Backward Reaching\nCCD: Cyclic Coordinate Descent\nASTAR: A* pathfinding (collision-aware)")
        self.ik_method_combo.currentTextChanged.connect(self.on_ik_method_changed)
        ik_method_layout.addWidget(self.ik_method_combo)
        solver_layout.addLayout(ik_method_layout)
        
        # A* Configuration button
        self.configure_astar_btn = QPushButton("Configure A* (Sample)")
        self.configure_astar_btn.clicked.connect(self.configure_astar)
        self.configure_astar_btn.setToolTip("Run sampling to configure A* pathfinding")
        self.configure_astar_btn.setEnabled(False)
        solver_layout.addWidget(self.configure_astar_btn)
        
        # Sampling method selection
        sampling_layout = QHBoxLayout()
        sampling_layout.addWidget(QLabel("Sampling:"))
        self.sampling_method_combo = QComboBox()
        self.sampling_method_combo.addItems(["Monte Carlo", "Grid"])
        self.sampling_method_combo.setCurrentText("Monte Carlo")
        self.sampling_method_combo.setToolTip("Monte Carlo: Random sampling\nGrid: Uniform grid with collision expansion")
        self.sampling_method_combo.currentTextChanged.connect(self.on_sampling_method_changed)
        sampling_layout.addWidget(self.sampling_method_combo)
        solver_layout.addLayout(sampling_layout)
        
        reset_btn = QPushButton("Reset Chain")
        reset_btn.clicked.connect(self.reset_chain)
        solver_layout.addWidget(reset_btn)
        
        solver_group.setLayout(solver_layout)
        layout.addWidget(solver_group)
        
        # Obstacles
        obstacle_group = QGroupBox("Obstacles")
        obstacle_layout = QVBoxLayout()
        
        obstacle_buttons = QHBoxLayout()
        self.add_obstacle_btn = QPushButton("Add Obstacle")
        self.add_obstacle_btn.setCheckable(True)
        self.add_obstacle_btn.clicked.connect(self.toggle_obstacle_mode)
        obstacle_buttons.addWidget(self.add_obstacle_btn)
        
        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self.clear_obstacles)
        obstacle_buttons.addWidget(clear_btn)
        obstacle_layout.addLayout(obstacle_buttons)
        
        radius_layout = QHBoxLayout()
        radius_layout.addWidget(QLabel("Radius:"))
        self.obstacle_radius_spin = QDoubleSpinBox()
        self.obstacle_radius_spin.setMinimum(10.0)
        self.obstacle_radius_spin.setMaximum(200.0)
        self.obstacle_radius_spin.setValue(50.0)
        self.obstacle_radius_spin.setSingleStep(5.0)
        radius_layout.addWidget(self.obstacle_radius_spin)
        obstacle_layout.addLayout(radius_layout)
        
        self.obstacle_count_label = QLabel("Obstacles: 0")
        obstacle_layout.addWidget(self.obstacle_count_label)
        
        note = QLabel("Click 'Add Obstacle', then click plot to place. Ctrl+drag to move.")
        note.setWordWrap(True)
        note.setStyleSheet("font-size: 9pt; color: gray;")
        obstacle_layout.addWidget(note)
        
        obstacle_group.setLayout(obstacle_layout)
        layout.addWidget(obstacle_group)
        
        # Info
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
            "• Ctrl + Drag: Move target/obstacles<br>"
            "• Drag: Pan view<br>"
            "• Scroll: Zoom<br>"
            "• Click Configuration Space Viewer to see Monte Carlo sampling"
        )
        instructions.setWordWrap(True)
        instructions.setStyleSheet("font-size: 9pt; padding: 5px;")
        layout.addWidget(instructions)
        
        layout.addStretch()
        return panel
    
    # Event handlers
    def on_mouse_click(self, event):
        """Handle mouse clicks"""
        if event.button() == Qt.MouseButton.LeftButton:
            pos = self.plot_widget.plotItem.vb.mapSceneToView(event.scenePos())
            click_pos = np.array([pos.x(), pos.y()])
            
            modifiers = QApplication.keyboardModifiers()
            ctrl_pressed = modifiers == Qt.KeyboardModifier.ControlModifier
            
            if ctrl_pressed:
                # Check if clicking on target
                if np.linalg.norm(click_pos - self.target_position) < 20:
                    self.dragging_target = True
                    return
                
                # Check if clicking on obstacle
                for obs in self.obstacles:
                    if obs.is_point_inside_warning(click_pos):
                        self.dragging_obstacle = obs
                        return
            
            # Normal click
            if self.obstacle_mode:
                self.add_obstacle_at_position(click_pos)
    
    def on_mouse_move(self, pos):
        """Handle mouse movement"""
        view_pos = self.plot_widget.plotItem.vb.mapSceneToView(pos)
        new_pos = np.array([view_pos.x(), view_pos.y()])
        
        modifiers = QApplication.keyboardModifiers()
        ctrl_pressed = modifiers == Qt.KeyboardModifier.ControlModifier
        
        if ctrl_pressed:
            if self.dragging_target:
                self.target_position = new_pos
                self.target_x_spinbox.setValue(new_pos[0])
                self.target_y_spinbox.setValue(new_pos[1])
                if self.auto_solve:
                    self.chain.solve(self.target_position)
            
            if self.dragging_obstacle:
                self.dragging_obstacle.position = new_pos
        else:
            self.dragging_target = False
            self.dragging_obstacle = None
    
    def on_joints_changed(self, value):
        """Handle joint count change"""
        current = self.chain.num_joints
        if value > current:
            for _ in range(value - current):
                self.add_joint()
        elif value < current:
            for _ in range(current - value):
                self.remove_joint()
    
    def on_target_changed(self):
        """Handle target position change"""
        self.target_position[0] = self.target_x_spinbox.value()
        self.target_position[1] = self.target_y_spinbox.value()
        if self.auto_solve:
            self.chain.solve(self.target_position)
    
    def on_display_changed(self):
        """Handle display options change"""
        self.show_reach_circle = self.show_reach_checkbox.isChecked()
    
    def on_auto_solve_changed(self):
        """Handle auto-solve toggle"""
        self.auto_solve = self.auto_solve_checkbox.isChecked()
    
    def on_ik_method_changed(self, method):
        """Handle IK method change"""
        self.chain.set_ik_method(method)
        print(f"IK method changed to: {method}")
        
        # Enable/disable A* configuration button
        self.configure_astar_btn.setEnabled(method == "ASTAR")
    
    def on_sampling_method_changed(self, method):
        """Handle sampling method change"""
        if method == "Monte Carlo":
            self.current_sampler = self.mc_sampler
            print("Switched to Monte Carlo sampling")
        elif method == "Grid":
            self.current_sampler = self.grid_sampler
            print("Switched to Grid sampling")
    
    def toggle_obstacle_mode(self):
        """Toggle obstacle placement mode"""
        self.obstacle_mode = self.add_obstacle_btn.isChecked()
        if self.obstacle_mode:
            self.add_obstacle_btn.setText("Adding... (click plot)")
        else:
            self.add_obstacle_btn.setText("Add Obstacle")
    
    def add_obstacle_at_position(self, position):
        """Add obstacle at specified position"""
        radius = self.obstacle_radius_spin.value()
        obstacle = Obstacle(position=position, radius=radius)
        self.obstacles.append(obstacle)
        self.chain.obstacles = self.obstacles
        
        # Create plot item
        circle_plot = pg.PlotDataItem(pen=pg.mkPen(color='r', width=2))
        self.plot_widget.addItem(circle_plot)
        self.obstacle_plots.append(circle_plot)
        
        self.obstacle_count_label.setText(f"Obstacles: {len(self.obstacles)}")
        
        # Turn off obstacle mode
        self.obstacle_mode = False
        self.add_obstacle_btn.setChecked(False)
        self.add_obstacle_btn.setText("Add Obstacle")
    
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()
        self.cspace_obstacles.clear()
        self.chain.obstacles = self.obstacles
        
        for plot in self.obstacle_plots:
            self.plot_widget.removeItem(plot)
        self.obstacle_plots.clear()
        
        if self.config_space_viewer and self.config_space_viewer.isVisible():
            self.config_space_viewer.clear_obstacle_regions()
        
        self.obstacle_count_label.setText("Obstacles: 0")
    
    def add_joint(self):
        """Add a joint to the chain"""
        link_length = self.link_length_spinbox.value()
        self.chain.add_joint(link_length)
        self.joints_spinbox.setValue(self.chain.num_joints)
        if self.config_space_viewer and self.config_space_viewer.isVisible():
            self.config_space_viewer.update_joint_count()
    
    def remove_joint(self):
        """Remove a joint from the chain"""
        if self.chain.num_joints > 1:
            self.chain.remove_joint()
            self.joints_spinbox.setValue(self.chain.num_joints)
            if self.config_space_viewer and self.config_space_viewer.isVisible():
                self.config_space_viewer.update_joint_count()
    
    def reset_chain(self):
        """Reset chain to straight line"""
        for i in range(self.chain.num_joints):
            self.chain.joints[i] = self.chain.base_position + np.array([0, i * 60])
        self.chain._update_joint_angles()
    
    def configure_astar(self):
        """Configure A* pathfinding by running Monte Carlo sampling"""
        sampler_name = self.sampling_method_combo.currentText()
        print(f"Running {sampler_name} sampling for A* configuration...")
        self.configure_astar_btn.setText("Sampling...")
        self.configure_astar_btn.setEnabled(False)
        QApplication.processEvents()  # Update UI
        
        # Clear previous configuration space data
        self.cspace_obstacles.clear()
        if self.config_space_viewer and self.config_space_viewer.isVisible():
            self.config_space_viewer.clear_obstacle_regions()
        
        # Run sampling once using selected sampler (considers ALL obstacles in chain)
        # This automatically considers ALL obstacles in self.chain.obstacles
        if self.obstacles:
            collision_configs, non_collision_configs, neighbor_map, endpos_map = self.current_sampler.sample_configuration_space(None)
            
            # Configure chain's A* with the sampled data
            self.chain.configure_astar(neighbor_map, non_collision_configs, endpos_map)
            print(f"A* configured with {len(non_collision_configs)} nodes")
            
            cspace_data = {
                'obstacle': self.obstacles[0] if len(self.obstacles) == 1 else None,  # For visualization
                'collision_configs': collision_configs,
                'non_collision_configs': non_collision_configs,
                'neighbor_map': neighbor_map,
                'endpos_map': endpos_map,
                'num_samples': len(collision_configs) + len(non_collision_configs)
            }
            self.cspace_obstacles.append(cspace_data)
            
            if self.config_space_viewer and self.config_space_viewer.isVisible():
                self.config_space_viewer.add_obstacle_region(cspace_data)
            
            print(f"A* configured with {sampler_name} sampling ({cspace_data['num_samples']} samples) "
                  f"considering {len(self.obstacles)} obstacle(s)")
            self.configure_astar_btn.setText("Configure A* (Reconfigure)")
        else:
            print("Warning: No obstacles present. A* works best with obstacles.")
            self.configure_astar_btn.setText("Configure A* (Reconfigure)")
        
        self.configure_astar_btn.setEnabled(True)
    
    def open_config_space_viewer(self):
        """Open configuration space viewer"""
        if self.config_space_viewer is None:
            self.config_space_viewer = ConfigSpaceViewer(self.chain, self)
        
        # Add existing obstacles
        for cspace_data in self.cspace_obstacles:
            self.config_space_viewer.add_obstacle_region(cspace_data)
        
        self.config_space_viewer.show()
    
    def open_joint_limits_dialog(self):
        """Open joint limits configuration dialog"""
        dialog = JointLimitsDialog(self.chain, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            print("Joint limits updated")
    
    def update_plot(self):
        """Update the visualization"""
        # Execute queued motion from A* if available
        if self.chain.has_queued_motion():
            self.chain.update()
        
        # Update chain lines
        x_coords = self.chain.joints[:, 0]
        y_coords = self.chain.joints[:, 1]
        self.links_plot.setData(x_coords, y_coords)
        
        # Update joints
        self.joints_plot.setData(x_coords[1:-1], y_coords[1:-1])
        
        # Update end effector
        self.end_effector_plot.setData([x_coords[-1]], [y_coords[-1]])
        
        # Update base
        self.base_plot.setData([x_coords[0]], [y_coords[0]])
        
        # Update target
        self.target_plot.setData([self.target_position[0]], [self.target_position[1]])
        
        # Update reach circle
        if self.show_reach_circle:
            theta = np.linspace(0, 2*np.pi, 100)
            radius = self.chain.total_length
            circle_x = self.chain.base_position[0] + radius * np.cos(theta)
            circle_y = self.chain.base_position[1] + radius * np.sin(theta)
            self.reach_circle.setData(circle_x, circle_y)
        else:
            self.reach_circle.setData([], [])
        
        # Update obstacles
        for i, obstacle in enumerate(self.obstacles):
            if i < len(self.obstacle_plots):
                theta = np.linspace(0, 2*np.pi, 100)
                circle_x = obstacle.position[0] + obstacle.radius * np.cos(theta)
                circle_y = obstacle.position[1] + obstacle.radius * np.sin(theta)
                self.obstacle_plots[i].setData(circle_x, circle_y)
        
        # Update angle limit visualizations
        self.update_angle_limits_visualization()
        
        # Update info
        distance = np.linalg.norm(self.target_position - self.chain.joints[-1])
        self.info_label.setText(
            f"End Effector: ({self.chain.joints[-1][0]:.1f}, {self.chain.joints[-1][1]:.1f})<br>"
            f"Target: ({self.target_position[0]:.1f}, {self.target_position[1]:.1f})<br>"
            f"Distance to target: {distance:.2f}<br>"
            f"Total reach: {self.chain.total_length:.1f}"
        )
        
        # Update config space viewer
        if self.config_space_viewer and self.config_space_viewer.isVisible():
            self.config_space_viewer.update()
    
    def update_angle_limits_visualization(self):
        """Update visual representation of joint angle limits"""
        # Ensure we have enough arc items
        while len(self.angle_limit_arcs) < self.chain.num_joints - 1:
            arc = pg.PlotDataItem(pen=pg.mkPen(color=(255, 165, 0, 100), width=1, style=Qt.PenStyle.DashLine))
            self.plot_widget.addItem(arc)
            self.angle_limit_arcs.append(arc)
        
        # Remove extra arc items if needed
        while len(self.angle_limit_arcs) > self.chain.num_joints - 1:
            arc = self.angle_limit_arcs.pop()
            self.plot_widget.removeItem(arc)
        
        # Update each arc to show angle limits
        for i in range(self.chain.num_joints - 1):
            joint_pos = self.chain.joints[i]
            link_length = self.chain.link_lengths[i]
            
            # Get angle limits for this joint
            min_angle, max_angle = self.chain.angle_limits[i]
            
            # For first joint, angles are absolute
            # For other joints, we need to calculate absolute angles from relative
            if i == 0:
                # First joint - angles are already absolute
                abs_min = min_angle
                abs_max = max_angle
            else:
                # Get the previous link's absolute angle
                prev_link_dir = self.chain.joints[i] - self.chain.joints[i-1]
                prev_angle = np.arctan2(prev_link_dir[1], prev_link_dir[0])
                
                # Convert relative limits to absolute
                abs_min = prev_angle + min_angle
                abs_max = prev_angle + max_angle
            
            # Create arc showing the valid angle range
            # Draw an arc from min to max angle
            num_points = 50
            angles = np.linspace(abs_min, abs_max, num_points)
            
            # Arc radius (slightly longer than link for visibility)
            arc_radius = link_length * 0.8
            
            arc_x = joint_pos[0] + arc_radius * np.cos(angles)
            arc_y = joint_pos[1] + arc_radius * np.sin(angles)
            
            self.angle_limit_arcs[i].setData(arc_x, arc_y)


class JointLimitsDialog(QDialog):
    """Dialog for configuring joint angle limits"""
    
    def __init__(self, chain, parent=None):
        super().__init__(parent)
        self.chain = chain
        self.setWindowTitle("Configure Joint Angle Limits")
        self.setModal(True)
        self.setMinimumWidth(500)
        
        layout = QVBoxLayout(self)
        
        # Instructions
        instructions = QLabel(
            "Configure angle limits for each joint. "
            "Joint 0 controls the first link's angle (absolute). "
            "Other joints control relative angles to the previous link."
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        
        # Create table
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(['Joint', 'Min Angle (°)', 'Max Angle (°)'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        
        # Populate table with current limits
        self.table.setRowCount(self.chain.num_joints)
        for i in range(self.chain.num_joints):
            # Joint label
            joint_label = QTableWidgetItem(f"Joint {i}")
            joint_label.setFlags(joint_label.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(i, 0, joint_label)
            
            # Min angle
            min_angle, max_angle = self.chain.angle_limits[i]
            min_spinbox = QDoubleSpinBox()
            min_spinbox.setMinimum(-360)
            min_spinbox.setMaximum(360)
            min_spinbox.setValue(np.degrees(min_angle))
            min_spinbox.setSuffix("°")
            self.table.setCellWidget(i, 1, min_spinbox)
            
            # Max angle
            max_spinbox = QDoubleSpinBox()
            max_spinbox.setMinimum(-360)
            max_spinbox.setMaximum(360)
            max_spinbox.setValue(np.degrees(max_angle))
            max_spinbox.setSuffix("°")
            self.table.setCellWidget(i, 2, max_spinbox)
        
        layout.addWidget(self.table)
        
        # Preset buttons
        preset_layout = QHBoxLayout()
        preset_label = QLabel("Presets:")
        preset_layout.addWidget(preset_label)
        
        full_range_btn = QPushButton("Full Range (0-180°)")
        full_range_btn.clicked.connect(self.apply_full_range)
        preset_layout.addWidget(full_range_btn)
        
        limited_btn = QPushButton("Limited (-90 to 90°)")
        limited_btn.clicked.connect(self.apply_limited_range)
        preset_layout.addWidget(limited_btn)
        
        preset_layout.addStretch()
        layout.addLayout(preset_layout)
        
        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        apply_btn = QPushButton("Apply")
        apply_btn.clicked.connect(self.apply_limits)
        button_layout.addWidget(apply_btn)
        
        ok_btn = QPushButton("OK")
        ok_btn.clicked.connect(self.accept_and_apply)
        button_layout.addWidget(ok_btn)
        
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(cancel_btn)
        
        layout.addLayout(button_layout)
    
    def apply_full_range(self):
        """Apply full range preset (0-180° for all joints)"""
        for i in range(self.chain.num_joints):
            min_spinbox = self.table.cellWidget(i, 1)
            max_spinbox = self.table.cellWidget(i, 2)
            if i == 0:
                min_spinbox.setValue(0)
                max_spinbox.setValue(180)
            else:
                min_spinbox.setValue(0)
                max_spinbox.setValue(180)
    
    def apply_limited_range(self):
        """Apply limited range preset (-90 to 90° for all joints)"""
        for i in range(self.chain.num_joints):
            min_spinbox = self.table.cellWidget(i, 1)
            max_spinbox = self.table.cellWidget(i, 2)
            min_spinbox.setValue(-90)
            max_spinbox.setValue(90)
    
    def apply_limits(self):
        """Apply the configured limits to the chain"""
        for i in range(self.chain.num_joints):
            min_spinbox = self.table.cellWidget(i, 1)
            max_spinbox = self.table.cellWidget(i, 2)
            
            min_angle = np.radians(min_spinbox.value())
            max_angle = np.radians(max_spinbox.value())
            
            self.chain.set_joint_limits(i, min_angle, max_angle)
        
        print(f"Applied angle limits for {self.chain.num_joints} joints")
    
    def accept_and_apply(self):
        """Apply limits and close dialog"""
        self.apply_limits()
        self.accept()


def main():
    app = QApplication([])
    widget = FABRIKWidget()
    widget.show()
    app.exec()


if __name__ == "__main__":
    main()
