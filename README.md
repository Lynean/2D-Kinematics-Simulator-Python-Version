# FABRIK Interactive Kinematic Chain Simulator

An advanced 2D inverse kinematics simulator featuring FABRIK (Forward And Backward Reaching Inverse Kinematics) algorithm with intelligent obstacle avoidance, collision detection, and smooth motion interpolation.

![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
![PyQt6](https://img.shields.io/badge/PyQt6-6.0+-green.svg)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

## 📋 Table of Contents

- [Features](#-features)
- [Installation](#-installation)
- [Usage](#-usage)
- [How It Works](#-how-it-works)
- [Project Structure](#-project-structure)
- [Configuration Options](#️-configuration-options)
- [Technical Details](#-technical-details)
- [Tips & Best Practices](#-tips--best-practices)
- [Known Limitations](#️-known-limitations)
- [Future Enhancements](#-future-enhancement-ideas)
- [Troubleshooting](#-troubleshooting)
- [License](#-license)

## 🎬 Quick Demo

```bash
# Quick start
git clone https://github.com/Lynean/2D-Kinematics-Simulator-Python-Version.git
cd 2D-Kinematics-Simulator-Python-Version/python_simulator
pip install PyQt6 pyqtgraph numpy
python main.py
```

**Try this**:
1. Drag the orange target around with `Ctrl + Left-click`
2. Click "Add Obstacle" and place some obstacles
3. Click "Start Drawing" and draw a path
4. Click "Follow Path" and watch the chain navigate!

## ✨ Features

### 🎯 Core Inverse Kinematics
- **FABRIK Algorithm**: Fast and efficient inverse kinematics solver
- **Real-time Solving**: 60 FPS smooth animation
- **Configurable Chain**: 2-20 joints with adjustable link lengths
- **Selectable End-Effector**: Choose any joint as the target point
- **Reachability Visualization**: Dynamic reach circle showing valid target area

### 🚧 Advanced Obstacle System
- **Interactive Obstacle Placement**: Click to add obstacles anywhere in the workspace
- **Dual-Zone Collision System**:
  - **Collision Radius** (solid red): Hard boundary - triggers collision detection
  - **Warning Zone** (dashed orange): Soft avoidance influences movement direction
- **Tangent-Based Navigation**: Links automatically route around obstacles during solving
- **Real-time Boundary Projection**: Prevents joints from penetrating obstacles

### 🛡️ Collision Detection & Response
- **Path Following Safety**: Automatically stops path following when collision detected
- **Joint-Level Detection**: Monitors all joints (except base) for obstacle penetration
- **Visual Feedback**: Clear collision warnings with automatic path stop
- **Manual Mode Support**: Collision avoidance during interactive dragging

### 🎬 Smooth Motion Interpolation
- **Angular Interpolation**: Servo-like smooth transitions between poses
- **Initial Waypoint Interpolation**: Smoothly moves to path start before following
- **Configurable Speed**: Adjustable interpolation speed (0.01-1.0)
- **Progress Tracking**: Real-time interpolation progress display

**⚠️ Known Limitation**: Manual target dragging near obstacles may cause snapping/jittering during collision avoidance. Smooth obstacle navigation is fully functional during automated path following.

### 🛤️ Intelligent Path Planning
- **Interactive Path Drawing**: Click to create custom paths (restricted to reachable area)
- **Smart Interpolation**: Configurable step size (5-50 pixels) for smooth waypoint generation
- **Automatic Path Following**: 
  - Smooth interpolation to first waypoint
  - Sequential waypoint progression
  - Automatic loop closure
  - Collision-triggered path stopping
- **Visual Feedback**: 
  - Cyan interpolated path points
  - Yellow raw waypoint markers
  - Current target highlighting

### 🎮 Interactive Controls
- **Multi-Mode Interaction**:
  - `Ctrl + Left-drag`: Move target, base, joints, or obstacles
  - `Left-drag`: Pan the view
  - `Right-drag`: Zoom box selection
  - `Mouse wheel`: Zoom in/out
- **Real-time Configuration**: Adjust parameters without stopping simulation
- **Joint Configuration Dialog**: Precise manual positioning via table editor
- **Joint Angles Monitor**: Live absolute and relative angle display

### 📊 Rich Visualization
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

## 🚀 Installation

### Prerequisites
- Python 3.8 or higher
- pip package manager

### Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Lynean/2D-Kinematics-Simulator-Python-Version.git
   cd 2D-Kinematics-Simulator-Python-Version/python_simulator
   ```

2. **Create virtual environment** (recommended):
   ```bash
   python -m venv venv
   ```

3. **Activate virtual environment**:
   - **Windows**:
     ```bash
     .\venv\Scripts\activate
     ```
   - **Linux/Mac**:
     ```bash
     source venv/bin/activate
     ```

4. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```
   
   Or manually:
   ```bash
   pip install PyQt6 pyqtgraph numpy
   ```

## 📖 Usage

### Running the Application
```bash
python main.py
```

### Basic Workflow

1. **Setup Chain**:
   - Adjust number of joints (2-20) using the spinbox
   - Set default link length (10-200 pixels)
   - Use "Add Joint" / "Remove Joint" for fine control
   - Select end-effector joint (any joint in the chain)

2. **Position Target**:
   - Enter coordinates manually in the spinboxes, or
   - `Ctrl + Left-drag` the orange triangle marker
   - Green dashed circle shows reachable area

3. **Add Obstacles**:
   - Click "Add Obstacle" button to enter placement mode
   - Click anywhere on the plot to place obstacles
   - Adjust obstacle radius (10-200 pixels) before placing
   - `Ctrl + Drag` existing obstacles to reposition
   - Click "Clear Obstacles" to remove all

4. **Draw & Follow Paths**:
   - Click "Start Drawing" to enter path mode
   - Click points inside the reach circle (green dashed line)
   - Adjust "Path Step Size" (5-50) for waypoint density
   - Click "Follow Path" to start automated movement
   - Path automatically stops if collision is detected
   - Yellow markers show raw waypoints, cyan shows interpolated path

5. **Configure Motion**:
   - **Smooth Speed**: Adjust interpolation speed (0.01-1.0)
     - Lower values = smoother, slower transitions
     - Higher values = faster, more direct movement
   - **Path Speed**: Movement speed along path (1-10)
   - Monitor interpolation progress in the info panel

6. **Monitor System**:
   - **Information Panel**: Real-time stats on chain, target, solver, and path
   - **Joint Configuration**: Click "Configure Joints" for manual positioning
   - **Angles Monitor**: Click "Show Joint Angles" for real-time angle display

### 🎮 Controls Reference

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
   - Check link-obstacle intersections → redirect along tangent
   - Check joint penetration → push to boundary
4. **Iteration**: Repeat until convergence or max iterations (10)
5. **Collision Check**: Verify final solution doesn't penetrate obstacles
6. **Return**: Success status or "COLLISION_DETECTED" string

### Path Following with Collision Safety

**Initial Interpolation**:
1. Solve FABRIK for first waypoint (without moving)
2. Set up smooth interpolation from current pose to first waypoint pose
3. Animate transition using angular interpolation
4. Begin path following once at start position

**Path Progression**:
1. **Target Assignment**: Current waypoint becomes target
2. **Solve**: Run FABRIK with collision avoidance
3. **Collision Check**: 
   - If `solve()` returns `"COLLISION_DETECTED"`: **STOP** path following
   - Display warning message
   - Keep chain at current safe position
4. **Convergence Check**: If end-effector within 5 pixels of waypoint
5. **Advance**: Move to next waypoint (loops to start)

### Obstacle Avoidance System

**Two-Zone Architecture**:
- **Collision Radius** (solid red circle): Hard boundary
  - Joints cannot penetrate - triggers path stop
  - Links are redirected around obstacles
  - Immediate boundary projection if violation occurs
  
- **Warning Zone** (dashed orange circle): Soft influence area
  - Affects link direction during FABRIK iterations
  - Provides gradual tangent-based steering
  - Creates smoother navigation paths

**Tangent-Based Navigation**:
- Links approaching obstacles are redirected along tangent vectors
- Prevents sudden directional changes
- Maintains link length constraints while avoiding collisions

## 📁 Project Structure

```
python_simulator/
├── main.py                # Application entry point
├── widget.py              # Main UI widget (FABRIKWidget)
├── src/
│   ├── chain.py          # FABRIKChain - Core IK solver with collision detection
│   ├── obstacle.py       # Obstacle - Collision zones and avoidance math
│   ├── path.py           # Path - Path management and interpolation
│   └── dialogs.py        # JointConfigDialog, JointAnglesDialog
├── requirements.txt       # Python dependencies
├── README.md             # This file
└── venv/                 # Virtual environment (not in git)
```

### Key Classes

- **`FABRIKChain`** (`src/chain.py`): 
  - Core FABRIK IK solver
  - Collision detection
  - Angular interpolation system
  - End-effector management

- **`Obstacle`** (`src/obstacle.py`): 
  - Dual-zone collision system
  - Tangent calculation for avoidance
  - Boundary projection
  - Point-inside-circle testing

- **`Path`** (`src/path.py`):
  - Path drawing and storage
  - Waypoint interpolation
  - Path following state management
  - Reachability validation

- **`FABRIKWidget`** (`widget.py`): 
  - Main application window
  - PyQtGraph visualization
  - User interaction handling
  - Real-time updates (60 FPS)

- **`JointConfigDialog`** (`src/dialogs.py`): 
  - Manual joint positioning
  - Table-based coordinate editing

- **`JointAnglesDialog`** (`src/dialogs.py`): 
  - Real-time angle monitoring
  - Absolute and relative angle display

## ⚙️ Configuration Options

### Chain Configuration
- Number of joints: 2-20
- Default link length: 10-200 pixels
- End-effector selection: Any joint (0 to n-1)
- Base position: Draggable or manual entry

### Target Control
- X/Y coordinates: -1000 to 1000
- Interactive positioning: `Ctrl + Left-drag`
- Auto-solve: Continuous IK solving toggle

### Display Options
- Show reach circle: Toggle end-effector reachability boundary
- Grid: Built-in with PyQtGraph
- Real-time information panel

### Solver Parameters
- Max iterations: 10 (hardcoded)
- Tolerance: 0.5 pixels (hardcoded)
- Smooth speed: 0.01-1.0 (default: 0.02)

### End Effector
- Select any joint (0 = base, N-1 = tip)
- Affects reach radius and target following
- Changes dynamically when dragging joints

### Path Settings
- Step size: 5-50 pixels (waypoint density)
- Path speed: 1-10 (movement speed multiplier)
- Drawing restriction: Inside reach circle only
- Automatic loop closure to first waypoint

### Obstacles
- Radius: 10-200 pixels (warning zone)
- Collision radius: Automatic (60% of main radius)
- Unlimited obstacles supported
- Draggable after placement

## 🔧 Technical Details

### Dependencies
- **PyQt6** (6.0+): Modern GUI framework
- **pyqtgraph** (latest): High-performance real-time plotting
- **NumPy** (latest): Efficient numerical computations

### Performance Characteristics
- **Frame Rate**: ~60 FPS (16ms update interval)
- **Solver Convergence**: Typically 3-7 iterations for simple cases
- **Interpolation Duration**: 
  - Speed 0.02: ~50 frames ≈ 0.83 seconds
  - Speed 0.10: ~10 frames ≈ 0.17 seconds
  - Speed 1.00: ~1 frame ≈ instant
- **Maximum Joints**: 20 (performance tested up to 15 smoothly)

### Coordinate System
- **Origin**: Top-left corner
- **Units**: Pixels
- **Y-axis**: Increases downward (standard screen coordinates)
- **Default workspace**: 600x600 pixels (scrollable/zoomable)

## 💡 Tips & Best Practices

### For Smooth Animation
- Set "Smooth Speed" to 0.01-0.03 for cinematic motion
- Use smaller path step sizes (5-15) for detailed following
- Enable "Show Reach Circle" to understand movement boundaries

### For Responsive Control
- Increase "Smooth Speed" to 0.3-1.0 for instant response
- Use larger path step sizes (30-50) for faster path traversal
- Toggle auto-solve OFF when manually configuring joints

### For Complex Obstacle Courses
- Start with fewer obstacles and add gradually
- Use smaller obstacle radii (20-50) for tighter navigation
- Test path following first, then add obstacles incrementally
- Monitor collision warnings in the info panel

### For Understanding Kinematics
- Keep "Joint Angles" monitor open while experimenting
- Try different end-effector selections (not just the tip!)
- Use "Configure Joints" to set up specific test poses
- Observe how reach radius changes with end-effector selection

## ⚠️ Known Limitations

### Current Limitations
- **2D only**: No 3D support (future enhancement possible)
- **No joint limits**: Full 360° rotation on all joints
- **No self-collision**: Chain can pass through itself
- **Manual mode snapping**: Dragging target near obstacles may cause jittering/snapping
  - ✅ **Works perfectly**: Automated path following with collision detection
  - ⚠️ **Known issue**: Manual dragging can be unstable near obstacles
- **Reactive navigation**: Obstacles detected during solving, not planned around

### Performance Considerations
- Large numbers of obstacles (>10) may slow solver convergence
- Very long chains (>15 joints) may impact frame rate
- Complex obstacle arrangements may cause local minima in IK solution

## 🚀 Future Enhancement Ideas

### Planned Improvements
- [ ] Joint angle constraints (servo-realistic min/max limits)
- [ ] Smooth manual mode obstacle navigation
- [ ] Self-collision detection and avoidance
- [ ] Save/load workspace configurations
- [ ] Export path animations (GIF/video)

### Potential Features
- [ ] 3D visualization mode
- [ ] A* pathfinding through obstacle fields
- [ ] Multi-chain support (multiple arms)
- [ ] Physics simulation (gravity, momentum, springiness)
- [ ] Inverse dynamics (force calculation)
- [ ] Keyboard shortcuts for common operations
- [ ] Preset scenes/demo configurations

## 🐛 Troubleshooting

### Application won't start
- **Check Python version**: `python --version` (need 3.8+)
- **Verify dependencies**: `pip list | grep -E "(PyQt6|pyqtgraph|numpy)"`
- **Reinstall packages**:
  ```bash
  pip uninstall PyQt6 pyqtgraph numpy
  pip install PyQt6 pyqtgraph numpy
  ```

### Chain won't reach target
- **Check reach circle**: Target must be inside green dashed boundary
- **Change end-effector**: Try selecting a different joint as end-effector
- **Obstacles blocking**: Remove obstacles temporarily to test
- **Increase tolerance**: Target might be just barely outside reach

### Jittery motion near obstacles
- **This is expected in manual mode**: Known limitation when dragging
- **Use path following instead**: Collision detection is smooth during automated paths
- **Reduce obstacle size**: Smaller collision zones = less interference
- **Increase smooth speed**: Faster interpolation reduces visible jitter

### Path following stops immediately
- **Collision detected**: First waypoint may be inside an obstacle
- **Clear obstacles**: Remove obstacles blocking the path
- **Redraw path**: Ensure all waypoints are in clear space
- **Check info panel**: "Following: No" indicates stopped state

### Poor performance/lag
- **Too many joints**: Reduce to 10 or fewer
- **Too many obstacles**: Remove some obstacles
- **Large path**: Increase step size to reduce waypoint count
- **Close other apps**: Free up system resources

## 📚 Algorithm References

### FABRIK (Forward And Backward Reaching Inverse Kinematics)
- **Paper**: "FABRIK: A fast, iterative solver for the Inverse Kinematics problem" by Andreas Aristidou and Joan Lasenby (2011)
- **Key advantages**: 
  - Fast convergence (typically <10 iterations)
  - No matrix inversions required
  - Handles multiple constraints naturally
  - Computationally efficient for real-time use

### Obstacle Avoidance
- **Tangent-based redirection**: Custom implementation for smooth navigation
- **Dual-zone collision**: Inspired by robotics proximity sensing
- **Boundary projection**: Standard computational geometry technique

## 📄 License

This project is released under the MIT License. See LICENSE file for details.

## 👨‍💻 Contributing

Contributions are welcome! Please feel free to:
- Report bugs via GitHub Issues
- Suggest features and enhancements
- Submit pull requests with improvements
- Share interesting use cases or demonstrations

## 🙏 Acknowledgments

- **FABRIK Algorithm**: Andreas Aristidou and Joan Lasenby
- **PyQt6**: The Qt Company and Riverbank Computing
- **pyqtgraph**: Luke Campagnola and contributors
- **NumPy**: NumPy development team

## 📞 Contact

For questions, suggestions, or collaboration:
- **GitHub**: [Lynean](https://github.com/Lynean)
- **Repository**: [2D-Kinematics-Simulator-Python-Version](https://github.com/Lynean/2D-Kinematics-Simulator-Python-Version)

---

**Enjoy exploring inverse kinematics!** 🎉🤖
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
