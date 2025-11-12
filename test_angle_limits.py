"""
Test script for joint angle constraints
"""
import numpy as np
from src.chain import FABRIKChain

# Create a simple 3-joint chain
chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)

print("=== Testing Joint Angle Constraints ===\n")

# Test 1: Check default limits
print("Test 1: Default Angle Limits")
for i in range(chain.num_joints):
    min_rad, max_rad = chain.angle_limits[i]
    min_deg = np.degrees(min_rad)
    max_deg = np.degrees(max_rad)
    print(f"Joint {i}: {min_deg:.1f}° to {max_deg:.1f}°")
print()

# Test 2: Set custom limits for joint 1 (90 to 180 degrees)
print("Test 2: Setting Custom Limits for Joint 1 (90° to 180°)")
chain.set_joint_limits(1, np.radians(90), np.radians(180))
min_rad, max_rad = chain.angle_limits[1]
print(f"Joint 1 limits: {np.degrees(min_rad):.1f}° to {np.degrees(max_rad):.1f}°")
print()

# Test 3: Test angle clamping
print("Test 3: Testing Angle Clamping for Joint 1")
test_angles = [0, 45, 90, 135, 180, 225, 270, 315]
for angle_deg in test_angles:
    angle_rad = np.radians(angle_deg)
    clamped_rad = chain.clamp_angle(angle_rad, 1)
    clamped_deg = np.degrees(clamped_rad)
    print(f"  Input: {angle_deg:3d}° → Clamped: {clamped_deg:6.1f}°")
print()

# Test 4: Solve with constraints
print("Test 4: Solving IK with Angle Constraints")

# First, reset to default limits
chain.set_joint_limits(1, np.radians(0), np.radians(180))

print("Initial configuration:")
print(f"  Joint positions: {chain.joints}")

# Calculate initial angles
for i in range(1, chain.num_joints):
    direction = chain.joints[i] - chain.joints[i-1]
    angle_rad = np.arctan2(direction[1], direction[0])
    
    if i > 1:
        prev_direction = chain.joints[i-1] - chain.joints[i-2]
        prev_angle = np.arctan2(prev_direction[1], prev_direction[0])
        relative_angle = (angle_rad - prev_angle) % (2 * np.pi)
    else:
        relative_angle = angle_rad % (2 * np.pi)
    
    print(f"  Joint {i} relative angle: {np.degrees(relative_angle):.1f}°")

# Set a target that's reachable
target = np.array([75, 75])
print(f"\nTarget: {target}")
print(f"Solving with default limits (0-180°)...")

result = chain.solve(target)
print(f"Result: {'Converged' if result else 'Max iterations reached'}")
print(f"Final positions:\n{chain.joints}")

# Calculate final angles
print("\nFinal angles:")
for i in range(1, chain.num_joints):
    direction = chain.joints[i] - chain.joints[i-1]
    angle_rad = np.arctan2(direction[1], direction[0])
    
    if i > 1:
        prev_direction = chain.joints[i-1] - chain.joints[i-2]
        prev_angle = np.arctan2(prev_direction[1], prev_direction[0])
        relative_angle = (angle_rad - prev_angle) % (2 * np.pi)
    else:
        relative_angle = angle_rad % (2 * np.pi)
    
    min_rad, max_rad = chain.angle_limits[i]
    within_limits = min_rad <= relative_angle <= max_rad
    
    print(f"Joint {i}: {np.degrees(relative_angle):.1f}° (limits: {np.degrees(min_rad):.1f}° to {np.degrees(max_rad):.1f}°) - {within_limits}")

print("\n=== Tests Complete ===")
