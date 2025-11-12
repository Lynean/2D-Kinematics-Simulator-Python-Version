"""
Test joint angle constraints with reachable targets
"""
import numpy as np
from src.chain import FABRIKChain

# Create a 4-joint chain
chain = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=40)

print("=== Testing Joint Angle Constraints with Reach able Targets ===\n")

# Test 1: No constraints (default 0-180°)
print("Test 1: Default constraints (0-180° for all joints except base)")
target = np.array([80, 80])
print(f"Target: {target}")

result = chain.solve(target)
print(f"Result: {'Converged' if result else 'Max iterations'}")
print(f"Iterations: {chain.current_iterations}\n")

# Show final angles
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
    
    print(f"Joint {i}: {np.degrees(relative_angle):.1f}° (limits: {np.degrees(min_rad):.1f}°-{np.degrees(max_rad):.1f}°) {'✓' if within_limits else '✗'}")

# Test 2: Very restrictive constraint on joint 2 (60-120°)
print("\n\nTest 2: Restrictive constraint on Joint 2 (60-120°)")
chain2 = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=40)
chain2.set_joint_limits(2, np.radians(60), np.radians(120))

target = np.array([80, 80])
print(f"Target: {target}")

result = chain2.solve(target)
print(f"Result: {'Converged' if result else 'Max iterations'}")
print(f"Iterations: {chain2.current_iterations}\n")

# Show final angles
for i in range(1, chain2.num_joints):
    direction = chain2.joints[i] - chain2.joints[i-1]
    angle_rad = np.arctan2(direction[1], direction[0])
    
    if i > 1:
        prev_direction = chain2.joints[i-1] - chain2.joints[i-2]
        prev_angle = np.arctan2(prev_direction[1], prev_direction[0])
        relative_angle = (angle_rad - prev_angle) % (2 * np.pi)
    else:
        relative_angle = angle_rad % (2 * np.pi)
    
    min_rad, max_rad = chain2.angle_limits[i]
    within_limits = min_rad <= relative_angle <= max_rad
    
    print(f"Joint {i}: {np.degrees(relative_angle):.1f}° (limits: {np.degrees(min_rad):.1f}°-{np.degrees(max_rad):.1f}°) {'✓' if within_limits else '✗'}")

# Test 3: Servo-like constraints (30-150° for all)
print("\n\nTest 3: Servo-like constraints (30-150° for all joints)")
chain3 = FABRIKChain(base_position=(0, 0), num_joints=5, link_length=30)
for i in range(1, chain3.num_joints):
    chain3.set_joint_limits(i, np.radians(30), np.radians(150))

target = np.array([100, 50])
print(f"Target: {target}")

result = chain3.solve(target)
print(f"Result: {'Converged' if result else 'Max iterations'}")
print(f"Iterations: {chain3.current_iterations}\n")

# Show final angles
for i in range(1, chain3.num_joints):
    direction = chain3.joints[i] - chain3.joints[i-1]
    angle_rad = np.arctan2(direction[1], direction[0])
    
    if i > 1:
        prev_direction = chain3.joints[i-1] - chain3.joints[i-2]
        prev_angle = np.arctan2(prev_direction[1], prev_direction[0])
        relative_angle = (angle_rad - prev_angle) % (2 * np.pi)
    else:
        relative_angle = angle_rad % (2 * np.pi)
    
    min_rad, max_rad = chain3.angle_limits[i]
    within_limits = min_rad <= relative_angle <= max_rad
    
    print(f"Joint {i}: {np.degrees(relative_angle):.1f}° (limits: {np.degrees(min_rad):.1f}°-{np.degrees(max_rad):.1f}°) {'✓' if within_limits else '✗'}")

print("\n=== All Tests Complete ===")
