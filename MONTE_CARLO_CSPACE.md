# Monte Carlo Configuration Space Obstacle Mapping

## Overview

The Monte Carlo C-space obstacle mapping feature automatically samples the robot's configuration space (joint space) to identify which joint configurations result in collisions with workspace obstacles. This provides valuable insight into the relationship between workspace obstacles and forbidden regions in configuration space.

## How It Works

### Monte Carlo Sampling Process

1. **Random Configuration Generation**: The system generates random joint angle configurations within the robot's constraint limits
2. **Forward Kinematics**: Each configuration is converted to workspace positions using forward kinematics
3. **Collision Detection**: The system checks if any link segments intersect with the obstacle
4. **Configuration Collection**: Configurations that result in collisions are stored
5. **Visualization**: Collision configurations are displayed in the Configuration Space Viewer

### Mathematical Details

For each sample:
- Generate random angles: `θᵢ ∈ [θᵢ_min, θᵢ_max]` for each joint i
- Compute link positions: `P = FK(θ₁, θ₂, ..., θₙ)`
- Check line-circle intersection for each link segment with obstacle circle
- If collision detected, store configuration `(θ₁, θ₂, ..., θₙ)`

## Features

### Automatic Integration

When you add an obstacle to the workspace:
1. Monte Carlo sampling runs automatically (1000 samples by default)
2. Collision configurations are computed in the background
3. Results are displayed in the Configuration Space Viewer
4. Console shows collision statistics

### Configuration Space Visualization

The Configuration Space Viewer displays obstacle regions:
- **2D Projection**: First two joints (Joint 0 vs Joint 1) shown as red scatter plot
- **Semi-transparent**: Red points with alpha transparency show collision density
- **Real-time Updates**: Obstacle regions update as you add/remove obstacles

### Toggle Button Mutual Exclusion

The following toggle buttons are now mutually exclusive:
- **Add Obstacle**: Place obstacles in the workspace
- **Draw Path**: Draw custom paths for the robot
- **A* Mode**: Use A* pathfinding with goal selection

When you activate one mode, the others automatically turn off, preventing mode conflicts.

## Usage

### Adding Obstacles with C-Space Mapping

1. Click the **"Add Obstacle"** button
2. Click anywhere in the workspace to place an obstacle
3. Monte Carlo sampling runs automatically
4. Open **"Configuration Space Viewer"** to see the forbidden regions

### Viewing C-Space Obstacles

1. Open the Configuration Space Viewer window
2. Look at the **2D Configuration Space** plot (bottom)
3. Red scatter points show configurations that collide with obstacles
4. Denser regions indicate higher collision probability

### Clearing Obstacles

1. Click **"Clear Obstacles"** to remove all obstacles
2. C-space obstacle regions are automatically cleared
3. Configuration Space Viewer updates to show only the valid region

## Technical Implementation

### MonteCarloSampler Class

Located in `src/monte_carlo.py`:

```python
class MonteCarloSampler:
    def __init__(self, chain, num_samples=1000):
        """Initialize with kinematic chain and sample count"""
        
    def sample_configuration_space(self, obstacle):
        """Sample random configurations and detect collisions"""
        
    def get_obstacle_region_2d(self, obstacle):
        """Get 2D projection for visualization"""
```

### Integration Points

**widget.py**:
- `MonteCarloSampler` initialized in `__init__`
- Sampling triggered in `mouse_pressed` when obstacle added
- C-space data stored in `self.cspace_obstacles` list
- Results passed to `ConfigSpaceViewer.add_obstacle_region()`

**config_space_viewer.py**:
- `add_obstacle_region()`: Display collision configurations as scatter plot
- `clear_obstacle_regions()`: Remove all obstacle visualizations
- Red semi-transparent points show forbidden configurations

## Performance

Sampling performance (5-joint robot, typical obstacle):

| Samples | Time  | Collisions | Accuracy |
|---------|-------|------------|----------|
| 100     | 0.01s | ~10        | Low      |
| 500     | 0.04s | ~40        | Medium   |
| 1000    | 0.08s | ~90        | Good     |
| 2000    | 0.16s | ~200       | High     |

**Default**: 1000 samples provides good balance of speed and accuracy.

## Examples

### Example 1: Single Obstacle

```python
# Create obstacle at (80, 60) with radius 30
obstacle = Obstacle(position=(80, 60), radius=30)

# Sample configuration space
mc_sampler = MonteCarloSampler(chain, num_samples=1000)
collision_configs = mc_sampler.sample_configuration_space(obstacle)

# Results: ~90 collision configurations out of 1000 samples
```

### Example 2: Multiple Obstacles

```python
obstacles = [
    Obstacle(position=(60, 50), radius=25),   # 137 collisions (13.7%)
    Obstacle(position=(100, 80), radius=20),  # 1 collision (0.1%)
    Obstacle(position=(40, -30), radius=15)   # 14 collisions (1.4%)
]

for obs in obstacles:
    collision_configs = mc_sampler.sample_configuration_space(obs)
    # Each obstacle creates a forbidden region in C-space
```

## Advantages

1. **Intuitive Visualization**: See how workspace obstacles map to joint space
2. **Educational**: Understand configuration space concepts
3. **Path Planning Aid**: Identify forbidden regions for better planning
4. **Debugging**: Diagnose why certain configurations fail
5. **Automatic**: No manual configuration required

## Limitations

1. **Approximation**: Monte Carlo provides approximate regions, not exact boundaries
2. **2D Visualization**: Only first two joints shown in 2D plot
3. **Computational Cost**: More samples = better accuracy but slower
4. **Memory**: Large sample counts consume more memory

## Configuration

You can adjust the number of samples in `widget.py`:

```python
# Default: 1000 samples
self.mc_sampler = MonteCarloSampler(self.chain, num_samples=1000)

# Higher accuracy (slower)
self.mc_sampler = MonteCarloSampler(self.chain, num_samples=2000)

# Faster (lower accuracy)
self.mc_sampler = MonteCarloSampler(self.chain, num_samples=500)
```

## Console Output

When adding an obstacle, you'll see:

```
Sampling C-space for obstacle at (80.0, 60.0) with radius 30
Found 93 collision configurations out of 1000 samples
Visualized 93 collision configs in C-space
```

This confirms:
- Obstacle position and size
- Number of detected collisions
- Sample count used
- Visualization status

## Future Enhancements

Potential improvements:
- Adjustable sample count in GUI
- 3D configuration space visualization
- Convex hull approximation of forbidden regions
- Parallel sampling for better performance
- Adaptive sampling (focus on boundaries)

## References

- Configuration Space: https://en.wikipedia.org/wiki/Configuration_space_(mathematics)
- Monte Carlo Methods: https://en.wikipedia.org/wiki/Monte_Carlo_method
- Collision Detection: Line-circle intersection geometry

## See Also

- `CONFIG_SPACE_VIEWER.md` - Configuration Space Viewer documentation
- `test_monte_carlo_integration.py` - Comprehensive test suite
- `src/monte_carlo.py` - Implementation details
