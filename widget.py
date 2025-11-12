from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QPushButton, QLabel, QSpinBox, 
                              QDoubleSpinBox, QGroupBox, QSlider, QCheckBox,
                              QDialog, QTableWidget, QTableWidgetItem, QHeaderView,
                              QScrollArea)
from PyQt6.QtCore import Qt, QTimer
import pyqtgraph as pg
import numpy as np

from src.chain import FABRIKChain
from src.dialogs import JointConfigDialog, JointAnglesDialog
from src.obstacle import Obstacle
from src.path import Path
from src.astar import AStarPathfinder

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
        self.target_position = self.chain.joints[-1].copy()
        
        # Interaction state
        self.show_reach_circle = True
        self.show_angle_limits = False  # Toggle angle limit visualization
        self.auto_solve = True
        self.manual_mode = False  # Flag to disable auto-solve during manual positioning
        
        # Obstacle mode
        self.obstacle_mode = False
        
        # A* Pathfinding
        self.astar = AStarPathfinder(grid_size=20)
        self.astar_mode = False  # A* pathfinding mode
        self.astar_goal = None  # Temporary goal for A* visualization
        self.show_grid = True  # Toggle grid visualization
        
        # Path manager
        self.path = Path(step_size=10.0, speed=2.0)
        
        self.setup_ui()
        
        # Timer for smooth updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(round(32/self.path.speed))  # ~30 FPS
    
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
        self.obstacle_plots = []  # List of (center_plot, circle_plot, warning_plot) tuples
        
        # A* visualization items
        self.grid_text_items = []  # List of TextItem objects for grid costs
        self.astar_path_plot = pg.PlotDataItem(pen=pg.mkPen(color='lime', width=3, style=Qt.PenStyle.SolidLine))
        self.astar_waypoints_plot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush('lime'), symbol='s')
        self.astar_goal_plot = pg.ScatterPlotItem(size=15, brush=pg.mkBrush('red'), symbol='x')
        
        # Angle limit visualization items
        self.angle_limit_plots = []  # List of PlotDataItem for angle limit arcs

        
        self.plot_widget.addItem(self.links_plot)
        self.plot_widget.addItem(self.joints_plot)
        self.plot_widget.addItem(self.end_effector_plot)
        self.plot_widget.addItem(self.base_plot)
        self.plot_widget.addItem(self.target_plot)
        self.plot_widget.addItem(self.reach_circle)
        self.plot_widget.addItem(self.path_plot)
        self.plot_widget.addItem(self.path_points_plot)
        self.plot_widget.addItem(self.raw_path_points_plot)
        self.plot_widget.addItem(self.astar_path_plot)
        self.plot_widget.addItem(self.astar_waypoints_plot)
        self.plot_widget.addItem(self.astar_goal_plot)
        
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
        
        self.show_angle_limits_checkbox = QCheckBox("Show Angle Limits")
        self.show_angle_limits_checkbox.setChecked(False)
        self.show_angle_limits_checkbox.stateChanged.connect(self.on_display_changed)
        display_layout.addWidget(self.show_angle_limits_checkbox)
        
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
        self.effector_spinbox.setValue(self.chain.end_effector_index)
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
        
        # Path interpolation toggle
        self.interpolate_path_checkbox = QCheckBox("Enable Path Interpolation")
        self.interpolate_path_checkbox.setChecked(True)
        self.interpolate_path_checkbox.setToolTip("Toggle smooth interpolation between path points. Uncheck to use raw points only.")
        self.interpolate_path_checkbox.stateChanged.connect(self.on_interpolation_changed)
        path_layout.addWidget(self.interpolate_path_checkbox)
        
        # Path smoothness control
        smooth_layout = QHBoxLayout()
        smooth_layout.addWidget(QLabel("Path Step Size:"))
        self.step_size_slider = QSlider(Qt.Orientation.Horizontal)
        self.step_size_slider.setMinimum(1)  # Allow minimum of 1 (fine interpolation)
        self.step_size_slider.setMaximum(50)
        self.step_size_slider.setValue(10)
        self.step_size_slider.setToolTip("Distance between path waypoints (1=finest, 50=coarsest)")
        self.step_size_slider.valueChanged.connect(self.on_step_size_changed)
        smooth_layout.addWidget(self.step_size_slider)
        self.step_size_label = QLabel("10")
        smooth_layout.addWidget(self.step_size_label)
        path_layout.addLayout(smooth_layout)
        
        path_group.setLayout(path_layout)
        layout.addWidget(path_group)
        
        # A* Pathfinding Controls
        astar_group = QGroupBox("A* Pathfinding")
        astar_layout = QVBoxLayout()
        
        # A* mode toggle
        self.astar_mode_btn = QPushButton("Enable A* Mode")
        self.astar_mode_btn.setCheckable(True)
        self.astar_mode_btn.clicked.connect(self.toggle_astar_mode)
        astar_layout.addWidget(self.astar_mode_btn)
        
        # Grid size control
        grid_size_layout = QHBoxLayout()
        grid_size_layout.addWidget(QLabel("Grid Size:"))
        self.grid_size_spin = QSpinBox()
        self.grid_size_spin.setMinimum(5)
        self.grid_size_spin.setMaximum(50)
        self.grid_size_spin.setValue(20)
        self.grid_size_spin.setSingleStep(5)
        self.grid_size_spin.valueChanged.connect(self.on_grid_size_changed)
        grid_size_layout.addWidget(self.grid_size_spin)
        grid_size_layout.addWidget(QLabel("px"))
        astar_layout.addLayout(grid_size_layout)
        
        # A* path interpolation step size
        astar_step_layout = QHBoxLayout()
        astar_step_layout.addWidget(QLabel("A* Step Size:"))
        self.astar_step_slider = QSlider(Qt.Orientation.Horizontal)
        self.astar_step_slider.setMinimum(5)
        self.astar_step_slider.setMaximum(50)
        self.astar_step_slider.setValue(10)
        self.astar_step_slider.valueChanged.connect(self.on_astar_step_changed)
        astar_step_layout.addWidget(self.astar_step_slider)
        self.astar_step_label = QLabel("10")
        astar_step_layout.addWidget(self.astar_step_label)
        astar_layout.addLayout(astar_step_layout)
        
        # Show grid toggle
        self.show_grid_checkbox = QCheckBox("Show Grid & Costs")
        self.show_grid_checkbox.setChecked(True)
        self.show_grid_checkbox.stateChanged.connect(self.on_show_grid_changed)
        astar_layout.addWidget(self.show_grid_checkbox)
        
        # Clear A* path
        clear_astar_btn = QPushButton("Clear A* Path")
        clear_astar_btn.clicked.connect(self.clear_astar_path)
        astar_layout.addWidget(clear_astar_btn)
        
        # Status label
        self.astar_status_label = QLabel("Status: Idle")
        self.astar_status_label.setStyleSheet("font-size: 9pt; color: gray;")
        astar_layout.addWidget(self.astar_status_label)
        
        astar_group.setLayout(astar_layout)
        layout.addWidget(astar_group)
        
        # Solver controls
        solver_group = QGroupBox("Solver Controls")
        solver_layout = QVBoxLayout()
        
        solve_btn = QPushButton("Solve Once")
        solve_btn.clicked.connect(self.solve_once)
        solver_layout.addWidget(solve_btn)
        
        reset_btn = QPushButton("Reset Chain")
        reset_btn.clicked.connect(self.reset_chain)
        solver_layout.addWidget(reset_btn)
        
        # Smooth interpolation toggle
        self.smooth_interp_checkbox = QCheckBox("Enable Smooth Movement")
        self.smooth_interp_checkbox.setChecked(True)
        self.smooth_interp_checkbox.setToolTip("Smooth angular interpolation between waypoints. Uncheck for instant movement.")
        self.smooth_interp_checkbox.stateChanged.connect(self.on_smooth_interp_changed)
        solver_layout.addWidget(self.smooth_interp_checkbox)
        
        # Interpolation speed control
        interp_speed_layout = QHBoxLayout()
        interp_speed_layout.addWidget(QLabel("Smooth Speed:"))
        self.interp_speed_spin = QDoubleSpinBox()
        self.interp_speed_spin.setMinimum(0.01)
        self.interp_speed_spin.setMaximum(1.0)
        self.interp_speed_spin.setValue(0.02)
        self.interp_speed_spin.setSingleStep(0.01)
        self.interp_speed_spin.setToolTip("Speed of smooth interpolation (0.01=slowest, 1.0=fastest)")
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
    
    def update_angle_limit_visualization(self):
        """Update visualization of angle limits for each joint"""
        # Clear existing angle limit plots
        for plot in self.angle_limit_plots:
            self.plot_widget.removeItem(plot)
        self.angle_limit_plots.clear()
        
        if not self.show_angle_limits:
            return
        
        # Draw angle limit arcs for each joint
        # Base joint (index 0) controls the first link's angle
        for i in range(self.chain.num_joints):
            # For base joint (i=0), we visualize the first link's angle constraint
            # For other joints (i>0), we visualize the angle at that joint
            
            if i == 0:
                # Base joint - visualize first link constraint
                min_angle, max_angle = self.chain.angle_limits[0]
                
                # Skip if full rotation (no constraint)
                if max_angle - min_angle >= 2 * np.pi - 0.01:
                    continue
                
                joint_pos = self.chain.joints[0]  # Base position
                
                # Absolute angles from horizontal
                abs_min = min_angle
                abs_max = max_angle
                
                # Arc radius for base
                arc_radius = self.chain.link_lengths[0] * 0.5 if len(self.chain.link_lengths) > 0 else 30
            else:
                # Regular joint constraint
                min_angle, max_angle = self.chain.angle_limits[i]
                
                # Skip if full rotation (no constraint)
                if max_angle - min_angle >= 2 * np.pi - 0.01:
                    continue
                
                # Get joint position
                joint_pos = self.chain.joints[i]
                
                # Get previous link angle (for relative angle calculation)
                prev_direction = joint_pos - self.chain.joints[i - 1]
                prev_angle = np.arctan2(prev_direction[1], prev_direction[0])
                
                # Calculate absolute angles
                abs_min = prev_angle + min_angle
                abs_max = prev_angle + max_angle
                
                # Arc radius (proportional to link length)
                if i < len(self.chain.link_lengths):
                    arc_radius = self.chain.link_lengths[i] * 0.5
                else:
                    arc_radius = 30
            
            # Generate arc points
            num_points = 50
            theta = np.linspace(abs_min, abs_max, num_points)
            arc_x = joint_pos[0] + arc_radius * np.cos(theta)
            arc_y = joint_pos[1] + arc_radius * np.sin(theta)
            
            # Add radial lines at limits
            line_start_x = [joint_pos[0], joint_pos[0] + arc_radius * np.cos(abs_min)]
            line_start_y = [joint_pos[1], joint_pos[1] + arc_radius * np.sin(abs_min)]
            line_end_x = [joint_pos[0], joint_pos[0] + arc_radius * np.cos(abs_max)]
            line_end_y = [joint_pos[1], joint_pos[1] + arc_radius * np.sin(abs_max)]
            
            # Create plot items
            arc_plot = pg.PlotDataItem(
                arc_x, arc_y,
                pen=pg.mkPen(color=(255, 200, 0, 150), width=2)
            )
            line1_plot = pg.PlotDataItem(
                line_start_x, line_start_y,
                pen=pg.mkPen(color=(255, 200, 0, 100), width=1)
            )
            line2_plot = pg.PlotDataItem(
                line_end_x, line_end_y,
                pen=pg.mkPen(color=(255, 200, 0, 100), width=1)
            )
            
            # Add to plot
            self.plot_widget.addItem(arc_plot)
            self.plot_widget.addItem(line1_plot)
            self.plot_widget.addItem(line2_plot)
            
            # Track for cleanup
            self.angle_limit_plots.append(arc_plot)
            self.angle_limit_plots.append(line1_plot)
            self.angle_limit_plots.append(line2_plot)

    
    def update_plot(self):
        """Update the plot with current chain state"""
        # Follow path if enabled
        if self.path.is_following and self.path.has_points():
            target = self.path.get_current_target()
            self.target_position = target
            
            # Solve for current waypoint
            if self.auto_solve and not self.manual_mode:
                result = self.solve_for_end_effector()
                
                # Check if collision was detected - stop path following
                if result == "COLLISION_DETECTED":
                    self.path.stop_following()
                    self.follow_path_btn.setChecked(False)
                    self.follow_path_btn.setText("Follow Path")
                    print("Collision Detected", 
                                      "Collision with obstacle detected during path following!\n"
                                      "Path following has been stopped.")
                    return
                
                # Check if close enough to current waypoint OR if max iterations reached without convergence
                distance_to_waypoint = np.linalg.norm(
                    self.chain.joints[self.chain.end_effector_index] - self.target_position
                )
                if distance_to_waypoint < 5.0 or not result:
                    # Move to next waypoint
                    self.path.advance_to_next_waypoint()
                    
                    # If smooth interpolation is enabled, start interpolating to the new waypoint
                    if self.chain.enable_smooth_interpolation:
                        next_waypoint = self.path.get_current_target()
                        
                        # Solve for the next waypoint to get target pose
                        if self.chain.end_effector_index == self.chain.num_joints - 1:
                            # Full chain case
                            temp_chain = FABRIKChain(self.chain.base_position,
                                                    num_joints=self.chain.num_joints,
                                                    obstacles=self.chain.obstacles)
                            temp_chain.joints = self.chain.joints.copy()
                            temp_chain.link_lengths = self.chain.link_lengths.copy()
                            temp_chain.solve(next_waypoint)
                            
                            # Start interpolation to next waypoint
                            self.chain.update_interpolation(poseA=self.chain.joints, poseB=temp_chain.joints)
                        else:
                            # Sub-chain case
                            temp_chain = FABRIKChain(self.chain.base_position,
                                                    num_joints=self.chain.end_effector_index + 1,
                                                    obstacles=self.chain.obstacles)
                            temp_chain.joints = self.chain.joints[:self.chain.end_effector_index + 1].copy()
                            temp_chain.link_lengths = self.chain.link_lengths[:self.chain.end_effector_index]
                            temp_chain.solve(next_waypoint)
                            
                            # Create full target with remaining joints in current positions
                            full_target_joints = self.chain.joints.copy()
                            full_target_joints[:self.chain.end_effector_index + 1] = temp_chain.joints
                            
                            # Start interpolation to next waypoint
                            self.chain.update_interpolation(poseA=self.chain.joints, poseB=full_target_joints)
                        
                        print(f"Interpolating to waypoint {self.path.current_index}...")

        else:
            # Only auto-solve if not in manual mode and not following path
            if self.auto_solve and not self.manual_mode:
                self.solve_for_end_effector()
        
        # Update links
        self.links_plot.setData(self.chain.joints[:, 0], self.chain.joints[:, 1])
        
        # Update joints (exclude end effector as it's shown separately)
        non_effector_joints = np.delete(self.chain.joints, self.chain.end_effector_index, axis=0)
        if len(non_effector_joints) > 0:
            self.joints_plot.setData(non_effector_joints[:, 0], non_effector_joints[:, 1])
        else:
            self.joints_plot.clear()
        
        # Update end effector (highlighted)
        effector_pos = self.chain.joints[self.chain.end_effector_index]
        self.end_effector_plot.setData([effector_pos[0]], [effector_pos[1]])
        
        # Update base
        self.base_plot.setData([self.chain.base_position[0]], [self.chain.base_position[1]])
        
        # Update target
        self.target_plot.setData([self.target_position[0]], [self.target_position[1]])
        
        # Calculate reach circle radius (distance from base to end effector)
        reach_radius = sum(self.chain.link_lengths[:self.chain.end_effector_index]) if self.chain.end_effector_index > 0 else 0
        
        # Update reach circle
        if self.show_reach_circle:
            theta = np.linspace(0, 2 * np.pi, 100)
            circle_x = self.chain.base_position[0] + reach_radius * np.cos(theta)
            circle_y = self.chain.base_position[1] + reach_radius * np.sin(theta)
            self.reach_circle.setData(circle_x, circle_y)
        else:
            self.reach_circle.clear()
        
        # Update path
        if self.path.has_points():
            path_array = np.array(self.path.interpolated_points)
            self.path_plot.setData(path_array[:, 0], path_array[:, 1])
            self.path_points_plot.setData(path_array[:, 0], path_array[:, 1])
        else:
            self.path_plot.clear()
            self.path_points_plot.clear()
        
        # Update raw path points (yellow markers for clicked points)
        if len(self.path.raw_points) > 0:
            raw_array = np.array(self.path.raw_points)
            self.raw_path_points_plot.setData(raw_array[:, 0], raw_array[:, 1])
        else:
            self.raw_path_points_plot.clear()
        
        # Update A* visualization
        if self.astar_mode and self.astar_goal is not None:
            # Show A* waypoints (green squares)
            if len(self.path.astar_waypoints) > 0:
                waypoints_array = np.array(self.path.astar_waypoints)
                self.astar_waypoints_plot.setData(waypoints_array[:, 0], waypoints_array[:, 1])
                # Draw path connecting waypoints
                self.astar_path_plot.setData(waypoints_array[:, 0], waypoints_array[:, 1])
            
            # Show goal marker
            self.astar_goal_plot.setData([self.astar_goal[0]], [self.astar_goal[1]])
            
            # Show grid costs if enabled
            if self.show_grid:
                self.update_grid_visualization()
        else:
            self.astar_waypoints_plot.clear()
            self.astar_path_plot.clear()
            self.astar_goal_plot.clear()
        
        # Update angle limit visualization
        self.update_angle_limit_visualization()
        
        # Update info
        distance = np.linalg.norm(self.target_position - self.chain.base_position)
        reachable = distance <= reach_radius
        end_effector = self.chain.joints[self.chain.end_effector_index]
        error = np.linalg.norm(end_effector - self.target_position)
        
        info_text = f"""
        <b>Chain Stats:</b><br>
        Joints: {self.chain.num_joints}<br>
        Total Length: {self.chain.total_length:.1f}<br>
        End Effector: Joint {self.chain.end_effector_index}<br>
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
        Smooth Movement: {'Enabled' if self.chain.enable_smooth_interpolation else 'Disabled (Instant)'}<br>
        Interpolation Speed: {self.chain.interpolation_speed:.2f}<br>
        Interpolating: {'Yes' if self.chain.is_interpolating else 'No'}<br>
        {f'Progress: {self.chain.interpolation_progress*100:.0f}%' if self.chain.is_interpolating else ''}<br>
        <br>
        <b>Path Info:</b><br>
        Points: {len(self.path.interpolated_points)}<br>
        Interpolation: {'Enabled' if self.path.enable_interpolation else 'Disabled (Raw Points)'}<br>
        Following: {'Yes' if self.path.is_following else 'No'}
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
        
        # If in A* pathfinding mode, set goal and generate path (no Ctrl needed)
        if self.astar_mode:
            # Check if inside reach circle
            reach_radius = sum(self.chain.link_lengths[:self.chain.end_effector_index]) if self.chain.end_effector_index > 0 else 0
            
            if self.path.is_point_in_reach([x, y], self.chain.base_position, reach_radius):
                self.generate_astar_path((x, y))
                event.accept()
            else:
                self.astar_status_label.setText("Status: Goal outside reach!")
            return
        
        # If drawing path, add point only if inside reach circle (no Ctrl needed)
        if self.path.is_drawing:
            # Calculate reach radius
            reach_radius = sum(self.chain.link_lengths[:self.chain.end_effector_index]) if self.chain.end_effector_index > 0 else 0
            
            if self.path.is_point_in_reach([x, y], self.chain.base_position, reach_radius):
                self.path.add_point([x, y])
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
                self.chain.end_effector_index = i
                self.mouse_pressed = True
                self.effector_spinbox.blockSignals(True)
                self.effector_spinbox.setValue(i)
                self.effector_spinbox.blockSignals(False)
                event.accept()
                return
    
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
        if self.chain.end_effector_index >= self.chain.num_joints:
            self.chain.end_effector_index = self.chain.num_joints - 1
            self.effector_spinbox.setValue(self.chain.end_effector_index)
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
        if self.chain.end_effector_index >= self.chain.num_joints:
            self.chain.end_effector_index = self.chain.num_joints - 1
            self.effector_spinbox.setValue(self.chain.end_effector_index)
        self.effector_note_label.setText("0 = Base, " + str(self.chain.num_joints - 1) + " = Tip")
    
    def on_target_changed(self):
        """Handle target position change from spinboxes"""
        self.target_position[0] = self.target_x_spinbox.value()
        self.target_position[1] = self.target_y_spinbox.value()
    
    def on_display_changed(self):
        """Handle display options change"""
        self.show_reach_circle = self.show_reach_checkbox.isChecked()
        self.show_angle_limits = self.show_angle_limits_checkbox.isChecked()
        self.update_plot()
    
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
            Mixed: True if converged, False if max iterations, "COLLISION_DETECTED" if collision detected
        """
        # Standard case - last joint is end effector
        result = self.chain.solve(self.target_position)
        return result  # Could be True, False, or "COLLISION_DETECTED"
    
    def on_effector_changed(self, value):
        """Handle end effector selection change"""
        self.chain.end_effector_index = value
    
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
        is_checked = self.draw_path_btn.isChecked()
        if is_checked:
            self.path.start_drawing()
            self.draw_path_btn.setText("Stop Drawing")
            self.path.stop_following()
            self.follow_path_btn.setChecked(False)
        else:
            self.path.stop_drawing()
            self.draw_path_btn.setText("Start Drawing")
            if self.path.has_points():
                self.follow_path_btn.setEnabled(True)
    
    def clear_path(self):
        """Clear the drawn path"""
        self.path.clear()
        self.follow_path_btn.setChecked(False)
        self.follow_path_btn.setEnabled(False)
    
    def toggle_follow_path(self):
        """Toggle path following mode"""
        is_checked = self.follow_path_btn.isChecked()
        if is_checked:
            self.path.start_following()
            self.follow_path_btn.setText("Stop Following")
            
            # Start interpolation to first waypoint before beginning path following
            first_waypoint = self.path.get_first_waypoint()
            if first_waypoint is not None:
                # Solve for the target pose at first waypoint (without applying it yet)
                # Create a temporary chain to compute the target configuration
                if self.chain.end_effector_index == self.chain.num_joints - 1:
                    # Full chain case
                    temp_chain = FABRIKChain(self.chain.base_position, 
                                            num_joints=self.chain.num_joints,
                                            obstacles=self.chain.obstacles)
                    temp_chain.joints = self.chain.joints.copy()
                    temp_chain.link_lengths = self.chain.link_lengths.copy()
                    temp_chain.solve(first_waypoint)
                    
                    # Start interpolation for full chain (if enabled)
                    if self.chain.enable_smooth_interpolation:
                        self.chain.update_interpolation(poseA = self.chain.joints, poseB=temp_chain.joints)
                        print("Starting interpolation to first waypoint...")
                    else:
                        # Directly set to target without interpolation
                        self.chain.joints = temp_chain.joints
                        print("Moving to first waypoint (no interpolation)...")
                else:
                    # Sub-chain case - only interpolate the sub-chain to end effector
                    temp_chain = FABRIKChain(self.chain.base_position,
                                            num_joints=self.chain.end_effector_index + 1,
                                            obstacles=self.chain.obstacles)
                    temp_chain.joints = self.chain.joints[:self.chain.end_effector_index + 1].copy()
                    temp_chain.link_lengths = self.chain.link_lengths[:self.chain.end_effector_index]
                    temp_chain.solve(first_waypoint)
                    
                    # For sub-chain, we need to create a full target_joints array
                    # that includes the remaining joints in their current positions
                    full_target_joints = self.chain.joints.copy()
                    full_target_joints[:self.chain.end_effector_index + 1] = temp_chain.joints
                    
                    # Start interpolation (if enabled)
                    if self.chain.enable_smooth_interpolation:
                        self.chain.update_interpolation(poseA = self.chain.joints, poseB=full_target_joints)
                        print("Starting interpolation to first waypoint...")
                    else:
                        # Directly set to target without interpolation
                        self.chain.joints = full_target_joints
                        print("Moving to first waypoint (no interpolation)...")

        else:
            self.path.stop_following()
            self.follow_path_btn.setText("Follow Path")
            # Stop interpolation if it's running
            if self.chain.is_interpolating:
                self.chain.is_interpolating = False
                self.chain.interpolation_progress = 0.0
                print("Stopped interpolation")
    
    def toggle_obstacle_mode(self):
        """Toggle obstacle placement mode"""
        self.obstacle_mode = self.add_obstacle_btn.isChecked()
        if self.obstacle_mode:
            self.add_obstacle_btn.setText("Placing... (click to add)")
            # Disable path drawing while in obstacle mode
            if self.path.is_drawing:
                self.path.stop_drawing()
                self.draw_path_btn.setChecked(False)
                self.draw_path_btn.setText("Draw Path")
        else:
            self.add_obstacle_btn.setText("Add Obstacle")
    
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()
    
    def on_speed_changed(self, value):
        """Handle path speed change"""
        self.path.set_speed(value)
        self.speed_label.setText(f"{value}.0")
        self.timer.start(round(64/self.path.speed))
    
    def on_interpolation_changed(self):
        """Handle path interpolation toggle"""
        self.path.enable_interpolation = self.interpolate_path_checkbox.isChecked()
        
        # Re-interpolate current path with new setting
        if self.path.is_astar_path:
            self.path.interpolate_astar()
        else:
            self.path.interpolate()
        
        # Update step size slider state
        self.step_size_slider.setEnabled(self.path.enable_interpolation)
        self.step_size_label.setEnabled(self.path.enable_interpolation)
    
    def on_smooth_interp_changed(self):
        """Handle smooth interpolation toggle"""
        self.chain.enable_smooth_interpolation = self.smooth_interp_checkbox.isChecked()
        
        # Enable/disable speed control based on smooth interpolation setting
        self.interp_speed_spin.setEnabled(self.chain.enable_smooth_interpolation)
        
        # If currently interpolating and user disabled it, stop interpolation
        if not self.chain.enable_smooth_interpolation and self.chain.is_interpolating:
            self.chain.is_interpolating = False
            self.chain.interpolation_progress = 0.0

    def on_step_size_changed(self, value):
        """Handle path step size change"""
        self.path.set_step_size(float(value))
        self.step_size_label.setText(str(value))
    
    def on_interp_speed_changed(self, value):
        """Handle interpolation speed change"""
        self.chain.interpolation_speed = value
    
    def toggle_astar_mode(self):
        """Toggle A* pathfinding mode"""
        self.astar_mode = self.astar_mode_btn.isChecked()
        if self.astar_mode:
            self.astar_mode_btn.setText("Disable A* Mode")
            self.astar_status_label.setText("Status: Click to set goal")
            
            # Disable other modes
            if self.path.is_drawing:
                self.path.stop_drawing()
                self.draw_path_btn.setChecked(False)
                self.draw_path_btn.setText("Start Drawing")
            
            if self.obstacle_mode:
                self.obstacle_mode = False
                self.add_obstacle_btn.setChecked(False)
                self.add_obstacle_btn.setText("Add Obstacle")
        else:
            self.astar_mode_btn.setText("Enable A* Mode")
            self.astar_status_label.setText("Status: Idle")
            self.astar_goal = None
    
    def on_grid_size_changed(self, value):
        """Handle grid size change"""
        self.astar.set_grid_size(value)
        # If we have an A* path, regenerate it with new grid size
        if self.astar_goal is not None:
            self.generate_astar_path(self.astar_goal)
    
    def on_astar_step_changed(self, value):
        """Handle A* interpolation step size change"""
        self.astar_step_label.setText(str(value))
        # If we have an A* path, re-interpolate it
        if self.path.is_astar_path and len(self.path.astar_waypoints) > 0:
            old_step = self.path.step_size
            self.path.step_size = float(value)
            self.path.interpolate_astar()
            # Don't change the regular path step size
    
    def on_show_grid_changed(self, state):
        """Handle show grid toggle"""
        self.show_grid = (state == Qt.CheckState.Checked.value)
    
    def generate_astar_path(self, goal_pos):
        """
        Generate A* path from current end-effector to goal
        
        Args:
            goal_pos: (x, y) tuple of goal position
        """
        # Get current end-effector position as start
        start_pos = self.chain.joints[self.chain.end_effector_index]
        
        # Update workspace bounds
        view_range = self.plot_widget.viewRange()
        self.astar.set_workspace_bounds(
            view_range[0][0], view_range[1][0],
            view_range[0][1], view_range[1][1]
        )
        
        # Generate obstacle grid
        self.astar.generate_obstacle_grid(self.obstacles)
        
        # Find path
        self.astar_status_label.setText("Status: Calculating path...")
        QApplication.processEvents()  # Update UI
        
        waypoints = self.astar.find_path(start_pos, goal_pos)
        
        if waypoints is None:
            self.astar_status_label.setText("Status: No path found!")
            self.astar_goal = None
            return
        
        # Set the A* path
        self.path.set_astar_path(waypoints)
        
        # Use A* step size for interpolation
        self.path.step_size = float(self.astar_step_slider.value())
        self.path.interpolate_astar()
        
        # Enable follow button
        self.follow_path_btn.setEnabled(True)
        
        self.astar_status_label.setText(f"Status: Path found! ({len(waypoints)} waypoints)")
        self.astar_goal = goal_pos
    
    def clear_astar_path(self):
        """Clear A* pathfinding data"""
        self.astar_goal = None
        self.astar.clear()
        if self.path.is_astar_path:
            self.path.clear()
            self.follow_path_btn.setEnabled(False)
        self.astar_status_label.setText("Status: Idle")
        # Clear grid text items
        for text_item in self.grid_text_items:
            self.plot_widget.removeItem(text_item)
        self.grid_text_items.clear()
    
    def update_grid_visualization(self):
        """Update grid cost visualization with text labels"""
        # Clear old text items
        for text_item in self.grid_text_items:
            self.plot_widget.removeItem(text_item)
        self.grid_text_items.clear()
        
        # Only show grid for cells that were visited during A*
        view_range = self.plot_widget.viewRange()
        min_x, max_x = view_range[0]
        min_y, max_y = view_range[1]
        
        # Limit number of text items for performance
        max_items = 200
        count = 0
        
        for grid_pos, cost in self.astar.grid_costs.items():
            if count >= max_items:
                break
            
            grid_x, grid_y = grid_pos
            world_x, world_y = self.astar.grid_to_world(grid_x, grid_y)
            
            # Only show if in view
            if min_x <= world_x <= max_x and min_y <= world_y <= max_y:
                # Create text item
                text = pg.TextItem(
                    text=f"{cost:.1f}",
                    color=(100, 100, 100),
                    anchor=(0.5, 0.5)
                )
                text.setFont(pg.QtGui.QFont("Arial", 8))
                text.setPos(world_x, world_y)
                
                self.plot_widget.addItem(text)
                self.grid_text_items.append(text)
                count += 1

