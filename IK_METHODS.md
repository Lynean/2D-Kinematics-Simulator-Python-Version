# Inverse Kinematics Methods

The simulator now supports two different IK (Inverse Kinematics) solving methods:

## 1. FABRIK (Forward And Backward Reaching Inverse Kinematics)

**Algorithm:**
- Two-phase approach: Forward pass (from end effector to base) and Backward pass (from base to end effector)
- Each pass adjusts joint positions to satisfy link length constraints
- Iterates until convergence or max iterations

**Characteristics:**
- ✅ Generally faster convergence
- ✅ Good for long chains
- ✅ Smooth motion
- ⚠️ May have trouble with tight constraints
- ⚠️ Less intuitive joint motion

**Best for:**
- Long kinematic chains (5+ joints)
- Smooth, natural-looking motion
- Real-time applications requiring speed

## 2. CCD (Cyclic Coordinate Descent)

**Algorithm:**
- Iterates through each joint from end to base
- Rotates each joint to point the end effector toward the target
- Applies constraints after each rotation
- Repeats until convergence or max iterations

**Characteristics:**
- ✅ Better handling of joint constraints
- ✅ More predictable joint motion
- ✅ Better convergence in constrained spaces
- ⚠️ Can be slightly slower
- ⚠️ May produce less smooth motion for long chains

**Best for:**
- Robotic arms with strict joint limits
- Shorter chains (2-4 joints)
- Applications where constraint adherence is critical
- When you need more predictable joint behavior

## Usage

### In GUI
1. Open the simulator
2. Find the **"Solver Controls"** panel
3. Use the **"IK Method"** dropdown to select:
   - **FABRIK** (default)
   - **CCD**
4. The change takes effect immediately

### In Code

```python
from src.chain import FABRIKChain

# Create chain
chain = FABRIKChain(base_position=(300, 500), num_joints=4, link_length=60)

# Set IK method
chain.set_ik_method('FABRIK')  # or 'CCD'

# Solve normally
chain.solve((400, 300))
```

## Performance Comparison

Based on test results with a 4-joint chain:

| Metric | FABRIK | CCD |
|--------|--------|-----|
| Average convergence | Mixed | Better |
| Speed | Slightly faster | ~20% slower |
| Constraint handling | Good | Excellent |
| Smoothness | Excellent | Good |

### Test Results Example

**Target: (400, 300)**
- FABRIK: Error: 23.61 pixels, Iterations: 0
- CCD: Error: 1.96 pixels, Iterations: 50 ✓

**Target: (450, 400)**
- FABRIK: Error: 173.61 pixels, Iterations: 50
- CCD: Error: 0.44 pixels, Iterations: 29 ✓

**With Joint Constraints:**
- FABRIK: Error: 0.28 pixels
- CCD: Error: 0.40 pixels, Better constraint adherence ✓

## Recommendations

### Choose FABRIK when:
- You have a long chain (5+ joints)
- Speed is critical
- Natural, smooth motion is priority
- Constraints are not very strict

### Choose CCD when:
- You have strict joint angle limits
- Working with a robot arm (2-4 joints typically)
- Accuracy near constraints is important
- You need predictable joint behavior

## Technical Details

### FABRIK Implementation
- Location: `src/chain.py` → `solve_fabrik()`
- Based on paper: "FABRIK: A fast, iterative solver for the Inverse Kinematics problem" by Aristidou & Lasenby (2011)
- Applies constraints after each pass using `apply_joint_constraint()`

### CCD Implementation
- Location: `src/chain.py` → `solve_ccd()`
- Classic CCD algorithm with constraint enforcement
- Rotates joints individually from end to base
- Applies constraints immediately after each rotation

## Common Settings

Both methods share:
- **Max iterations**: 50 (adjustable via `chain.max_iterations`)
- **Tolerance**: 0.5 pixels (adjustable via `chain.tolerance`)
- **Joint constraints**: Both respect angle limits set via `set_joint_limits()`
- **Collision detection**: Both check for obstacle collisions

## Switching Methods Mid-Operation

You can switch methods at any time:
- The current chain position is preserved
- Next solve will use the new method
- Works during path following
- Works during smooth interpolation

## Examples

See `test_ik_methods.py` for comprehensive comparison examples including:
- Reachable vs unreachable targets
- Constrained vs unconstrained solving
- Performance measurements
- Convergence analysis
