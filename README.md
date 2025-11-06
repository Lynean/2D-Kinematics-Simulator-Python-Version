# FABRIK Interactive Kinematic Chain Simulator

An advanced 2D inverse kinematics simulator featuring FABRIK (Forward And Backward Reaching Inverse Kinematics) algorithm with intelligent obstacle avoidance and smooth collision-aware motion interpolation.

![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
![PyQt6](https://img.shields.io/badge/PyQt6-6.0+-green.svg)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

## Features

### 🎯 Core Inverse Kinematics
- **FABRIK Algorithm**: Fast and efficient inverse kinematics solver
- **Real-time Solving**: 60 FPS smooth animation
- **Configurable Chain**: 2-20 joints with adjustable link lengths
- **Selectable End-Effector**: Choose any joint as the target point
- **Reachability Visualization**: Visual feedback for target reachability

### 🚧 Advanced Obstacle Avoidance
- **Hard Collision Detection**: Prevents joints and links from passing through obstacles
- **Dual-Zone System**:
  - **Collision Radius** (solid red): Hard boundary - no penetration allowed
  - **Warning Zone** (dashed orange): Soft avoidance influences movement direction
- **Tangent-Based Redirection**: Links automatically route around obstacles during FABRIK solving
- **Real-time Collision Response**: Instant boundary projection for penetrating joints

### 🎬 Intelligent Motion Smoothing
- **Snap Detection**: Identifies when IK solution changes drastically
- **Angular Interpolation**: Smooth servo-like transitions between poses
- **Collision-Aware Interpolation**: Automatically reverses joint angles when collisions occur during animation
- **Adaptive Resolution Sequence**: Propagating pattern (C→B→C→B→A→...) intelligently navigates around obstacles
- **Configurable Speed**: Adjustable interpolation speed (0.01-1.0)

### 🛤️ Path Planning
- **Interactive Path Drawing**: Click to create paths (restricted to reachable area)
- **Smooth Interpolation**: Configurable step size (5-50 pixels) for fluid motion
- **Path Following**: Automatic waypoint progression with convergence detection
- **Visual Feedback**: Cyan interpolated points, yellow waypoints

### 🎮 Interactive Controls
- **Drag & Drop**:
  - `Ctrl + Left-drag`: Move target, base, joints, or obstacles
  - `Left-drag`: Pan the view
  - `Right-drag`: Zoom box
  - `Mouse wheel`: Zoom in/out
- **Real-time Configuration**: Adjust chain, solver, and obstacle parameters on-the-fly
- **Joint Configuration Dialog**: Manually position joints with table-based editing
- **Joint Angles Monitor**: Real-time display of absolute and relative angles

### 📊 Visualization
- **Color-Coded Elements**:
  - Blue lines: Chain links
  - Red circles: Regular joints
  - Magenta star: Selected end-effector
  - Green square: Fixed base
  - Orange triangle: Target position
  - Green dashed circle: Reach boundary
  - Red solid circle: Obstacle collision zone
  - Orange dashed circle: Obstacle warning zone
- **Information Panel**: Real-time stats on chain, target, solver, and path
- **Scrollable UI**: All controls accessible in organized, scrollable sidebar

## Installation

### Prerequisites
- Python 3.8 or higher
- pip package manager

### Setup

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd python_simulator
   ```

2. **Create virtual environment** (recommended):
   ```bash
   python -m venv venv
   ```

3. **Activate virtual environment**:
   - Windows:
     ```bash
     .\venv\Scripts\activate
     ```
   - Linux/Mac:
     ```bash
     source venv/bin/activate
     ```

4. **Install dependencies**:
   ```bash
   pip install PyQt6 pyqtgraph numpy
   ```

## Usage

### Running the Application
```bash
python fabrik_app.py
```

### Basic Workflow

1. **Setup Chain**:
   - Adjust number of joints (2-20)
   - Set default link length (10-200)
   - Add/remove joints as needed

2. **Position Target**:
   - Enter coordinates manually, or
   - `Ctrl + Left-drag` the orange triangle

3. **Add Obstacles** (optional):
   - Click "Add Obstacle" button
   - Click on plot to place obstacles
   - Adjust warning radius (10-200)
   - `Ctrl + drag` to move obstacles

4. **Draw Path** (optional):
   - Click "Draw Path"
   - Click points inside reach circle
   - Adjust path step size for smoothness
   - Click "Follow Path" to animate

5. **Configure Solver**:
   - **Snap Threshold**: Higher = less sensitive to pose changes (5-100)
   - **Smooth Speed**: Lower = smoother transitions (0.01-1.0)
   - Monitor interpolation progress and reversals

6. **Monitor Angles**:
   - Click "Show Joint Angles" for real-time angle display
   - View absolute angles (world frame) and relative angles (servo offsets)

### Controls Reference

| Action | Control |
|--------|---------|
| Move objects (target/base/joints/obstacles) | `Ctrl + Left-drag` |
| Pan view | `Left-drag` |
| Zoom box | `Right-drag` |
| Zoom in/out | `Mouse wheel` |
| Add path point | Click (while in path drawing mode) |
| Place obstacle | Click (while in obstacle mode) |

## Algorithm Details

### FABRIK (Forward And Backward Reaching Inverse Kinematics)

1. **Forward Pass**: Start from end-effector, work backward to base
2. **Backward Pass**: Start from base, work forward to end-effector
3. **Collision Handling**: 
   - Check link-circle intersections → redirect along tangent
   - Check joint penetration → push to boundary
4. **Iteration**: Repeat until convergence or max iterations (10)

### Collision-Aware Interpolation

When transitioning between poses with obstacle collisions:

1. **Detect**: Check if interpolated pose collides with obstacles
2. **Identify**: Find furthest joint from base involved in collision
3. **Generate Sequence**: Create reversal pattern (e.g., C→B→C→B→A→...)
4. **Apply**: Reverse angle direction (go "long way around" - 360°)
5. **Iterate**: Apply next reversal in sequence each frame until resolved

**Example**: For collision at joint D in chain A-B-C-D-E:
- Frame 1: Reverse D (instead of +45°, go -315°)
- Frame 2: Reverse C
- Frame 3: Toggle D back
- Frame 4: Reverse B
- Pattern continues until collision clears

### Obstacle Avoidance

**Two-Zone System**:
- **Collision Radius** (60% of warning radius): Hard boundary
  - Joints cannot penetrate
  - Links cannot pass through
  - Immediate boundary projection if violation
  
- **Warning Zone** (100% radius): Soft influence
  - Affects movement direction during FABRIK
  - Blends normal direction with tangent based on penetration
  - Formula: `adjusted = (1-p)*normal + p*(0.7*tangent + 0.3*radial)`

## Project Structure

```
python_simulator/
├── fabrik_app.py          # Main application (all-in-one)
├── README.md              # This file
├── venv/                  # Virtual environment (not in git)
└── requirements.txt       # Python dependencies (if created)
```

### Key Classes

- **`Obstacle`**: Manages collision zones and avoidance calculations
- **`JointConfigDialog`**: Manual joint positioning interface
- **`JointAnglesDialog`**: Real-time angle monitoring display
- **`FABRIKChain`**: Core IK solver with collision handling
- **`FABRIKWidget`**: Main application window and visualization

## Configuration Options

### Chain Configuration
- Number of joints: 2-20
- Default link length: 10-200 pixels
- Base position: Set via drag or coordinate entry

### Target Control
- X/Y coordinates: Manual entry
- Drag positioning: `Ctrl + Left-drag`
- Auto-solve: Toggle for continuous solving

### Display Options
- Show reach circle: Toggle visibility
- Grid: Built-in with PyQtGraph

### Solver Parameters
- Max iterations: 10 (default)
- Tolerance: 0.5 pixels (default)
- Snap threshold: 5-100 (default: 30)
- Smooth speed: 0.01-1.0 (default: 0.02)

### End Effector
- Select any joint (0 = base, N-1 = tip)
- Affects reach radius and target following

### Path Settings
- Step size: 5-50 pixels (smoothness control)
- Drawing restriction: Inside reach circle only
- Follow mode: Automatic waypoint progression

### Obstacles
- Warning radius: 10-200 pixels
- Collision radius: Automatic (60% of warning)
- Unlimited obstacles supported

## Technical Details

### Dependencies
- **PyQt6**: GUI framework
- **pyqtgraph**: Real-time plotting and visualization
- **NumPy**: Mathematical operations and arrays

### Performance
- **Frame Rate**: ~60 FPS (16ms update interval)
- **Solver Speed**: Typically converges in 3-7 iterations
- **Interpolation**: 0.02 progress per frame (default) = ~50 frames = ~0.83s

### Coordinate System
- Origin: Top-left (PyQtGraph default)
- Units: Pixels
- Y-axis: Increases downward

## Tips & Tricks

1. **Smooth Motion**: Lower the "Smooth Speed" (0.01-0.02) for cinematic transitions
2. **Fast Response**: Increase "Smooth Speed" (0.3-1.0) for snappy movements
3. **Complex Paths**: Use smaller "Path Step Size" (5-10) for detailed following
4. **Obstacle Navigation**: Place multiple obstacles to create maze-like challenges
5. **Joint Monitoring**: Keep "Joint Angles" window open to understand pose changes
6. **Manual Posing**: Use "Configure Joint Positions" to set up starting poses
7. **Collision Testing**: Set snap threshold low (5-10) to see interpolation reversals frequently

## Known Limitations

- 2D only (no 3D support)
- No joint angle limits (full 360° rotation)
- No self-collision detection
- Path following doesn't account for future obstacles
- Large obstacle fields may cause convergence issues

## Future Enhancements

- [ ] Joint angle constraints (min/max limits)
- [ ] Self-collision avoidance
- [ ] 3D visualization option
- [ ] Save/load configurations
- [ ] Export animations
- [ ] A* path planning through obstacle fields
- [ ] Multi-chain support
- [ ] Physics simulation (gravity, momentum)

## Troubleshooting

**Application won't start**:
- Ensure Python 3.8+ is installed
- Check all dependencies are installed: `pip list`
- Try reinstalling PyQt6: `pip install --upgrade PyQt6`

**Chain won't reach target**:
- Check if target is inside reach circle (green dashed line)
- Verify end-effector index is correct
- Remove blocking obstacles

**Slow performance**:
- Reduce number of joints (< 10)
- Remove unnecessary obstacles
- Increase solver tolerance

**Interpolation looks jerky**:
- Lower the "Smooth Speed" value
- Reduce snap threshold to trigger smoothing more often
- Check that auto-solve is enabled

## License

MIT License - Feel free to use, modify, and distribute.

## Contributing

Contributions are welcome! Areas for improvement:
- Performance optimization
- Additional IK algorithms (CCD, Jacobian-based)
- Enhanced collision detection
- Better UI/UX
- Unit tests

## Acknowledgments

- FABRIK algorithm: Aristidou, A., & Lasenby, J. (2011)
- PyQtGraph library for excellent real-time plotting
- PyQt6 for robust cross-platform GUI framework

## Contact

For questions, issues, or suggestions, please open an issue on the repository.

---

**Enjoy exploring inverse kinematics with intelligent obstacle avoidance!** 🦾✨
