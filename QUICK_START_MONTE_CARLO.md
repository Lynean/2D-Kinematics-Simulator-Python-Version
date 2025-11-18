# Quick Start: Monte Carlo C-Space Obstacle Mapping

## What is this?

When you add an obstacle to the workspace, the system automatically:
1. Tests 1000 random robot configurations
2. Finds which configurations cause collisions
3. Shows the "forbidden region" in Configuration Space

## How to Use (3 Steps)

### Step 1: Add an Obstacle
1. Click **"Add Obstacle"** button
2. Click anywhere in the workspace
3. Watch the console for statistics

Example console output:
```
Sampling C-space for obstacle at (80.0, 60.0) with radius 30
Found 93 collision configurations out of 1000 samples
```

### Step 2: Open Configuration Space Viewer
1. Click **"Configuration Space Viewer"** button
2. A new window opens with two plots

### Step 3: See the Forbidden Region
Look at the **bottom plot** (2D Configuration Space):
- **Green rectangle**: Valid joint angle combinations
- **Red dots**: Forbidden configurations (robot hits obstacle)
- **Blue line**: Your current path through C-space

## What You're Seeing

### Top Plot: Time Series
- Shows all joint angles changing over time
- Each colored line is a different joint

### Bottom Plot: 2D C-Space (Joint 0 vs Joint 1)
- X-axis: Joint 0 angle
- Y-axis: Joint 1 angle
- Green box: Valid angles
- Red dots: Angles that cause collision
- Blue line: Path traced by robot

## Toggle Buttons

The following buttons are **mutually exclusive** (only one can be active):

| Button | What It Does |
|--------|--------------|
| **Add Obstacle** | Place obstacles in workspace |
| **Draw Path** | Draw custom path for robot |
| **Enable A* Mode** | Use A* pathfinding |

When you click one, the others turn off automatically.

## Tips

1. **Dense Red Regions**: High collision probability in that area
2. **Sparse Red Dots**: Low collision probability
3. **No Red Dots**: Obstacle too far away or very small
4. **Clear Obstacles**: Removes all obstacles and clears red dots

## Understanding the Numbers

Example: "Found 93 collision configurations out of 1000 samples"
- **1000 samples**: System tested 1000 random configurations
- **93 collisions**: 93 of those configurations hit the obstacle
- **Collision rate**: 93/1000 = 9.3%

Higher percentage = obstacle blocks more of the workspace.

## Performance

Typical timing:
- **1000 samples**: ~0.08 seconds
- **Adds to obstacle creation**: Minimal delay
- **Real-time**: Yes, happens automatically

## Troubleshooting

**Problem**: No red dots appear
- **Cause**: Obstacle too far from robot reach
- **Solution**: Place obstacle closer to robot base

**Problem**: Too many red dots (hard to see)
- **Cause**: Large obstacle or many obstacles
- **Solution**: Clear obstacles and add smaller ones

**Problem**: Config viewer doesn't update
- **Cause**: Window closed or not opened
- **Solution**: Click "Configuration Space Viewer" again

## Example Workflow

1. Start the application
2. Click "Add Obstacle"
3. Click at position (100, 100) in workspace
4. See console: "Found X collision configurations"
5. Click "Configuration Space Viewer"
6. See red forbidden region in bottom plot
7. Move robot and watch blue path avoid red regions
8. Add more obstacles to see how C-space changes

## What This Helps With

✅ **Understand** which joint angles to avoid  
✅ **Visualize** obstacle effects in joint space  
✅ **Debug** why certain movements fail  
✅ **Learn** configuration space concepts  
✅ **Plan** better paths around obstacles

## Learn More

- `MONTE_CARLO_CSPACE.md` - Full documentation
- `demo_monte_carlo.py` - Run demo: `python demo_monte_carlo.py`
- `test_monte_carlo_integration.py` - See test examples

## Key Concept

**Workspace**: Where the robot's links are (X, Y positions)  
**Configuration Space**: The robot's joint angles (θ₁, θ₂, etc.)  

An obstacle in workspace creates a **forbidden region** in C-space.

This tool shows you that forbidden region automatically! 🎯
