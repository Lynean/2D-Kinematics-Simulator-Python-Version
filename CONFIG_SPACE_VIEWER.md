# Configuration Space / Joint Space Viewer

A new visualization window that displays the robot's joint angles in configuration space, showing how the joint angles change over time and their relationship to joint constraints.

## Features

### 1. **Time Series View**
- Displays each joint's angle over time
- Color-coded lines for each joint (Joint 0, Joint 1, Joint 2, etc.)
- Real-time tracking of current joint configuration
- Configurable history length (10-2000 steps)

### 2. **2D Configuration Space View**
- Shows Joint 0 vs Joint 1 angles in a 2D plot
- Visualizes valid configuration region (green rectangle)
- Shows constraint boundaries for both joints
- Tracks path through configuration space
- Current position highlighted in red

### 3. **Controls**
- **Track Path**: Enable/disable angle history tracking
- **Clear History**: Reset all tracked data
- **Max History**: Adjust how many steps to remember
- Real-time status display showing current angles

## How to Use

### Opening the Viewer

1. Run the main application
2. In the **Chain Configuration** section, click **"Configuration Space Viewer"**
3. A new window will open showing the configuration space

### Understanding the Display

#### Top Plot: Time Series
- **X-axis**: Time steps (frame number)
- **Y-axis**: Joint angles in degrees
- **Colored lines**: Each color represents a different joint
- **Red dots**: Current angle for each joint
- **Legend**: Shows which color corresponds to which joint

#### Bottom Plot: 2D Configuration Space
- **X-axis**: Joint 0 angle (degrees)
- **Y-axis**: Joint 1 angle (degrees)
- **Green rectangle**: Valid configuration region (within joint limits)
- **Blue line**: Path traced through configuration space
- **Red circle**: Current configuration position

### Tracking Configuration Changes

The viewer automatically updates as you:
- Move the target position
- Follow a path
- Manually adjust joints
- Run IK solver

### Managing History

- **Enable/Disable Tracking**: Toggle "Track Path" checkbox
  - When disabled, history stops growing but remains visible
  - When re-enabled, continues from current position

- **Adjust History Length**: Use "Max History" spinner
  - Minimum: 10 steps
  - Maximum: 2000 steps
  - Default: 500 steps
  - Automatically trims old data when limit reached

- **Clear History**: Click "Clear History" button
  - Removes all tracked angles
  - Resets both plots
  - Useful when starting a new task

## Use Cases

### 1. **Understanding Joint Behavior**
Watch how individual joints contribute to reaching different targets:
```
- Draw a circular path in workspace
- Open Config Space Viewer
- Follow the path
- Observe sinusoidal patterns in joint angles
```

### 2. **Analyzing Constraint Violations**
See when joints approach or hit their limits:
```
- Set tight joint limits
- Move target near workspace boundary
- Watch angles approach constraint boundaries in 2D view
```

### 3. **Comparing IK Methods**
Compare FABRIK vs CCD behavior in configuration space:
```
- Clear history
- Use FABRIK method to reach target
- Note the path in config space
- Clear history
- Switch to CCD method
- Reach same target
- Compare the two paths
```

### 4. **Path Planning Analysis**
Understand how the robot moves through joint space along a path:
```
- Draw a complex path with A* pathfinding
- Enable tracking
- Follow path
- Observe joint angle trajectories
- Identify smooth vs jerky transitions
```

### 5. **Debugging Joint Issues**
Identify problems with joint configuration:
```
- If end effector doesn't reach target
- Check config space viewer
- See if joints are hitting limits
- Adjust constraints or target accordingly
```

## Technical Details

### What is Configuration Space?

Configuration space (C-space) represents all possible states of the robot:
- **Dimension**: Each joint adds one dimension
- **Point in C-space**: One specific configuration (set of joint angles)
- **Path in C-space**: Sequence of configurations over time
- **Constraints**: Joint limits define valid region

### Advantages of C-space Visualization

1. **Intuitive**: See joint angles directly, not just end effector position
2. **Constraint awareness**: Visualize valid vs invalid configurations
3. **Motion analysis**: Understand how joints coordinate
4. **Debugging**: Quickly identify constraint violations
5. **Learning**: Educational tool for understanding IK

### Limitations

- 2D view only shows first two joints
- High-dimensional spaces (many joints) harder to visualize
- Time series can get cluttered with many joints
- History limited to prevent memory issues

## Integration with Main App

The viewer is fully integrated:
- **Automatic updates**: Syncs with main simulation
- **Joint changes**: Updates when joints added/removed
- **Multiple windows**: Can open/close independently
- **No performance impact**: Updates only when visible

## Tips and Tricks

### Smooth Visualization
- Enable "Smooth Movement" in main app
- Slower interpolation speed shows clearer patterns
- Adjust Max History to zoom in/out on time axis

### Analyzing Periodicity
- Follow circular path in workspace
- Watch for repeating patterns in joint angles
- Helps understand robot's kinematic redundancy

### Finding Optimal Configurations
- Clear history before each attempt
- Try different approaches to same target
- Compare path lengths in configuration space
- Shorter paths often mean smoother motion

### Debugging Path Following
- If robot gets stuck during path following
- Check config space for constraint violations
- Look for discontinuities in angle plots
- May indicate IK solver switching between solutions

## Keyboard Shortcuts

While the viewer window is active:
- **Ctrl+W**: Close window
- **Escape**: Close window

## Data Export (Future Feature)

Potential enhancements:
- Export angle history to CSV
- Save configuration space snapshots
- Record and replay motion sequences
- Statistical analysis of joint usage

## Related Features

- **Joint Angles Monitor**: Shows current angles as table
- **Joint Configuration**: Set angle limits and positions
- **Path Drawing**: Create paths to follow
- **A* Pathfinding**: Generate obstacle-avoiding paths

## Example Scenarios

### Scenario 1: Circle Drawing
```python
# Draw circle in workspace → observe in config space
1. Clear history
2. Draw circular path
3. Follow path
4. Result: Sinusoidal angle patterns
```

### Scenario 2: Constraint Testing
```python
# Test joint limit behavior
1. Set Joint 1 limit: -45° to 45°
2. Move target requiring >45°
3. Observe: Angle stops at boundary in 2D view
```

### Scenario 3: IK Comparison
```python
# Compare FABRIK vs CCD paths
FABRIK:
  - Reach target
  - Note: Often uses larger joint motions
  
CCD:
  - Reach same target  
  - Note: Often more incremental changes
```

## Troubleshooting

**Viewer doesn't update:**
- Ensure "Track Path" is checked
- Check if window is visible
- Try clearing history and re-enabling tracking

**History grows too fast:**
- Reduce Max History value
- Disable tracking when not needed
- Clear history periodically

**2D view empty:**
- Need at least 2 joints in chain
- Ensure tracking is enabled
- Check if angles are within plot range

**Performance issues:**
- Reduce Max History (500 → 100)
- Close viewer when not needed
- Clear history before long operations

## Code Reference

- **File**: `src/config_space_viewer.py`
- **Class**: `ConfigSpaceViewer`
- **Integration**: `widget.py` → `open_config_space_viewer()`
- **Test**: `test_config_space.py`

## Future Enhancements

Planned improvements:
1. 3D configuration space view (for 3-joint systems)
2. Velocity and acceleration plots
3. Interactive constraint editing
4. Configuration space planning tools
5. Multi-chain comparison mode
6. Export to animation formats
