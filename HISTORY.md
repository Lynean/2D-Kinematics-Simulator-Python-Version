# Version History

## Version 1.5.0 - November 12, 2025
### Collision-Aware Interpolation & Speed Control

**New Features:**
- **Collision Detection During Interpolation**: Smooth interpolation now checks for obstacle collisions in real-time
- **Tangential Avoidance During Interpolation**: When collision detected during interpolation, the chain uses tangential avoidance to navigate around obstacles
- **Interpolation Speed Display**: Added interpolation speed to the info panel

**Improvements:**
- Enhanced `update_interpolation()` method to check each link segment for collision
- Applied tangential algorithm during interpolation to redirect links around obstacles
- Joints are pushed to obstacle boundaries if they enter collision zones
- Link lengths are maintained when applying collision avoidance during interpolation

**Technical Details:**
- Modified `chain.py::update_interpolation()` to include collision checking
- Uses existing `obstacle.get_tangent_direction()` for collision response
- Reconstructs subsequent joints after collision adjustment to maintain chain integrity

---

## Version 1.4.0 - November 12, 2025
### Complete Smooth Movement System

**New Features:**
- **Smooth Interpolation Between All Waypoints**: Chain now smoothly interpolates between every waypoint in a path, not just the first one
- **Interpolation Speed Control**: Added spinbox to control interpolation speed (0.01 = slowest, 1.0 = fastest)
- **Smooth Movement Toggle**: Checkbox to enable/disable smooth angular interpolation

**Improvements:**
- Modified `update_plot()` to start interpolation when advancing to each new waypoint
- When reaching a waypoint, if smooth interpolation is enabled, the chain solves for the next waypoint and starts interpolating
- If smooth interpolation is disabled, chain uses instant solving (previous behavior)
- Interpolation speed control automatically disabled when smooth movement is off

**Technical Details:**
- Added `chain.enable_smooth_interpolation` flag
- Modified waypoint advancement logic in `widget.py::update_plot()` (lines ~595-630)
- Interpolation triggered for both full chain and sub-chain cases
- Console message: "Interpolating to waypoint N..." when starting interpolation

---

## Version 1.3.0 - November 12, 2025
### Path Interpolation Control

**New Features:**
- **Path Step Interpolation Toggle**: Added checkbox to enable/disable interpolation between path waypoints
- **Minimum Path Step Size**: Allowed minimum path step size to be set to 1 pixel

**Improvements:**
- Added `path.enable_interpolation` flag to control path density
- Modified `path.interpolate()` and `path.interpolate_astar()` to respect interpolation flag
- When disabled, path uses raw waypoints only
- When enabled, path creates smooth interpolated waypoints

**Bug Fixes:**
- Added safety check in `path.get_current_target()` to prevent index out of bounds
- Fixed issue where changing interpolation mode during path following could crash

**Technical Details:**
- Added `enable_interpolation` parameter to Path class
- Modified path interpolation methods with conditional logic
- Added bounds checking: `min(self.current_index, len(self.interpolated_points) - 1)`

---

## Version 1.2.0 - November 12, 2025
### Base Joint Angle Constraints

**New Features:**
- **Base Joint Angle Limits**: Added configurable angle constraints for the base joint (first link)
- Base joint angle controls the first link's angle from horizontal

**Improvements:**
- Modified angle constraint system to support base joint limits
- Base joint (index 0) uses `angle_limits[0]` for constraint
- Default base joint range: 0° to 180° (0 to π radians)
- Other joints default: -90° to 90° (-π/2 to π/2 radians)

**Technical Details:**
- Modified `apply_joint_constraint()` to handle base joint specially
- Base joint constraint checks parent-to-joint angle relative to horizontal
- Other joints check angle relative to parent link direction
- Post-FABRIK constraint enforcement ensures limits are respected

**Code Changes:**
- Enhanced `chain.py::apply_joint_constraint()` with base joint logic
- Added `limit_index` calculation: `limit_index = joint_index if joint_index == 0 else joint_index - 1`

---

## Version 1.1.0 (Earlier)
### Enhanced Path Following & Visualization

**Features:**
- Path following with automatic waypoint advancement
- Visual feedback for path following state
- Distance-based waypoint completion (< 5.0 threshold)
- Convergence-based waypoint advancement

---

## Version 1.0.0 (Earlier)
### Core FABRIK Implementation

**Initial Features:**
- FABRIK (Forward And Backward Reaching Inverse Kinematics) solver
- Multi-joint chain support
- Obstacle avoidance with collision detection
- Tangential avoidance algorithm
- Interactive target positioning
- Real-time visualization with PyQt6
- A* pathfinding for obstacle-rich environments
- Joint angle constraints system
- Link length customization
- End effector selection

**Core Components:**
- `chain.py`: FABRIK solver with obstacle avoidance
- `obstacle.py`: Obstacle class with collision and warning zones
- `path.py`: Path management and interpolation
- `widget.py`: PyQt6 UI and visualization
- `main.py`: Application entry point

**Key Features:**
- Forward and backward reaching phases
- Joint angle limit enforcement
- Collision detection (point and link-based)
- Tangential direction calculation for avoidance
- Smooth servo-like angular interpolation
- Interactive drawing tools
- Real-time solver statistics display

---

## Architecture Overview

### Main Components:
1. **FABRIKChain** (`chain.py`): IK solver with constraints and collision avoidance
2. **Obstacle** (`obstacle.py`): Collision zones and tangent calculations
3. **Path** (`path.py`): Waypoint management and interpolation
4. **KinematicsWidget** (`widget.py`): UI, visualization, and user interaction

### Key Algorithms:
- **FABRIK**: Two-phase iterative IK solving
- **Joint Constraints**: Angle clamping with wraparound handling
- **Collision Avoidance**: Multi-stage detection and response
- **Tangential Avoidance**: Geometric tangent calculation for smooth obstacle navigation
- **Angular Interpolation**: Servo-like smooth movement between poses
- **A* Pathfinding**: Grid-based obstacle-aware path planning

### Data Flow:
1. User draws path → Path waypoints generated
2. Path interpolation (if enabled) → Dense waypoint list
3. Path following starts → Target set to first waypoint
4. FABRIK solver → IK solution with collision checking
5. Smooth interpolation (if enabled) → Angular interpolation between poses
6. Waypoint reached → Advance and interpolate to next waypoint
7. Repeat until path complete

---

## Configuration Parameters

### Chain Settings:
- `num_joints`: Number of joints in chain (default: 3)
- `link_length`: Length of each link (default: 50)
- `tolerance`: Convergence threshold (default: 0.5)
- `max_iterations`: Maximum solver iterations (default: 50)
- `interpolation_speed`: Smooth movement speed 0.01-1.0 (default: 0.02)

### Path Settings:
- `enable_interpolation`: Toggle path step interpolation (default: True)
- `path_step_size`: Minimum distance between waypoints (default: 5.0, min: 1.0)
- `astar_step_size`: A* grid resolution (default: 10)

### Obstacle Settings:
- `radius`: Warning zone radius (default: 50.0)
- `collision_radius`: Hard collision radius (default: radius * 0.6)

---

## Known Issues & Limitations
- Interpolation collision avoidance may struggle with very dense obstacle fields
- High interpolation speeds (>0.5) may reduce smoothness
- Very small path step sizes (<2) may impact performance
- Base joint constraints only enforced after FABRIK iterations

---

## Future Enhancements
- [ ] Dynamic obstacle support
- [ ] Multiple end effector modes
- [ ] Save/load configurations
- [ ] Path recording and playback
- [ ] Advanced collision response strategies
- [ ] GPU acceleration for large chains
- [ ] 3D visualization mode

---

## Credits
Developed using:
- Python 3.x
- PyQt6 for UI
- NumPy for numerical computations
- FABRIK algorithm for inverse kinematics
