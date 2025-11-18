# Simplified FABRIK Simulator

## Running the Simplified Version

Run the simplified application with:
```bash
python main_simple.py
```

## What's Included (Main Features)

### ✅ Chain Configuration
- Add/remove joints (0-4 joints)
- Adjust link length
- View in Configuration Space Viewer

### ✅ Target Control
- Drag target with Ctrl + Mouse
- Set exact position with spinboxes
- Auto-solve IK (FABRIK algorithm)

### ✅ Display Options
- Toggle reach circle
- Auto-solve on/off
- Reset chain to straight line

### ✅ Obstacles
- Add obstacles by clicking on plot
- Adjust obstacle radius
- Drag obstacles with Ctrl + Mouse
- Clear all obstacles
- **Monte Carlo sampling** - automatically samples configuration space when obstacle is added
- **Neighbor map** - builds connectivity graph for path planning

### ✅ Configuration Space Viewer
- View joint angles in real-time
- 2D configuration space plot (Joint 0 vs Joint 1)
- **Click on datapoints** to set robot configuration
- See collision vs safe configurations (from Monte Carlo)
- Grey dots = safe configurations
- Red dots = collision configurations

## What's Removed (Advanced Features)

### ❌ Path Drawing
- Draw path mode
- Follow path mode
- Path interpolation controls

### ❌ A* Pathfinding
- A* mode toggle
- Grid visualization
- Automatic path planning

### ❌ Complex Solver Controls
- IK method selection (FABRIK/CCD)
- Smooth interpolation controls
- End effector selection

### ❌ Dialog Windows
- Joint configuration dialog
- Joint angles monitor

## Controls

- **Ctrl + Drag**: Move target or obstacles
- **Drag**: Pan the view
- **Scroll**: Zoom in/out
- **Click**: Place obstacles (when "Add Obstacle" is active)

## File Structure

- `main_simple.py` - Entry point for simplified version
- `widget_simple.py` - Simplified UI (350 lines vs 1350 lines)
- `main.py` - Original full-featured version
- `widget.py` - Original full-featured UI

## Key Benefits of Simplified Version

1. **Easier to understand** - 75% less code
2. **Faster to load** - No unnecessary features
3. **Cleaner UI** - Only essential controls
4. **Perfect for learning** - Focus on core IK concepts
5. **Still powerful** - Includes Monte Carlo sampling and Configuration Space visualization

## Technical Features Still Working

- FABRIK inverse kinematics
- Relative angle system (first absolute, rest relative)
- Joint angle caching
- Obstacle collision detection
- Monte Carlo configuration space sampling
- Neighbor map for graph-based planning
- Real-time visualization updates

---

**Use `main_simple.py` for a clean experience** or `main.py` if you need advanced features like path planning and A* pathfinding.
