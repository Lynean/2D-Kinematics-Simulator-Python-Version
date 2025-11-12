"""
Test base joint angle constraints
"""
import numpy as np
from src.chain import FABRIKChain

print("=== Testing Base Joint Angle Constraints ===\n")

# Create a simple 3-joint chain
chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)

print("Test 1: Default Base Constraint (0-180°)")
print(f"Base angle limits: {np.degrees(chain.angle_limits[0][0]):.1f}° to {np.degrees(chain.angle_limits[0][1]):.1f}°\n")

# Try to reach a target at 45 degrees (should work)
target1 = np.array([70, 70])
print(f"Target 1 (northeast): {target1}")
result1 = chain.solve(target1)
print(f"Result: {'Converged' if result1 else 'Max iterations'}")

# Calculate first link angle
first_link_direction = chain.joints[1] - chain.joints[0]
first_link_angle = np.arctan2(first_link_direction[1], first_link_direction[0])
first_link_angle_deg = np.degrees(first_link_angle % (2 * np.pi))
print(f"First link angle: {first_link_angle_deg:.1f}°")
print(f"Within limits: {0 <= first_link_angle_deg <= 180}\n")

# Try to reach a target below horizontal (should be clamped)
target2 = np.array([70, -70])
print(f"Target 2 (southeast, below horizontal): {target2}")
result2 = chain.solve(target2)
print(f"Result: {'Converged' if result2 else 'Max iterations'}")

first_link_direction = chain.joints[1] - chain.joints[0]
first_link_angle = np.arctan2(first_link_direction[1], first_link_direction[0])
first_link_angle_deg = np.degrees(first_link_angle % (2 * np.pi))
print(f"First link angle: {first_link_angle_deg:.1f}°")
print(f"Within limits: {0 <= first_link_angle_deg <= 180}")
print(f"(Should be clamped to 0° or 180°)\n")

# Test 2: Restrict base to 45-135 degrees
print("\nTest 2: Restricted Base (45-135°)")
chain2 = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
chain2.set_joint_limits(0, np.radians(45), np.radians(135))
print(f"Base angle limits: {np.degrees(chain2.angle_limits[0][0]):.1f}° to {np.degrees(chain2.angle_limits[0][1]):.1f}°\n")

# Try horizontal target (should be clamped to 45°)
target3 = np.array([100, 0])
print(f"Target 3 (horizontal): {target3}")
result3 = chain2.solve(target3)
print(f"Result: {'Converged' if result3 else 'Max iterations'}")

first_link_direction = chain2.joints[1] - chain2.joints[0]
first_link_angle = np.arctan2(first_link_direction[1], first_link_direction[0])
first_link_angle_deg = np.degrees(first_link_angle % (2 * np.pi))
print(f"First link angle: {first_link_angle_deg:.1f}°")
print(f"Within limits: {45 <= first_link_angle_deg <= 135}")
print(f"(Should be clamped to 45°)\n")

# Try vertical target (should work)
target4 = np.array([0, 100])
print(f"Target 4 (vertical): {target4}")
result4 = chain2.solve(target4)
print(f"Result: {'Converged' if result4 else 'Max iterations'}")

first_link_direction = chain2.joints[1] - chain2.joints[0]
first_link_angle = np.arctan2(first_link_direction[1], first_link_direction[0])
first_link_angle_deg = np.degrees(first_link_angle % (2 * np.pi))
print(f"First link angle: {first_link_angle_deg:.1f}°")
print(f"Within limits: {45 <= first_link_angle_deg <= 135}\n")

# Test 3: Very restricted base (80-100 degrees) - narrow range
print("\nTest 3: Very Narrow Base Range (80-100°)")
chain3 = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=40)
chain3.set_joint_limits(0, np.radians(80), np.radians(100))
print(f"Base angle limits: {np.degrees(chain3.angle_limits[0][0]):.1f}° to {np.degrees(chain3.angle_limits[0][1]):.1f}°\n")

target5 = np.array([50, 100])
print(f"Target 5: {target5}")
result5 = chain3.solve(target5)
print(f"Result: {'Converged' if result5 else 'Max iterations'}")

first_link_direction = chain3.joints[1] - chain3.joints[0]
first_link_angle = np.arctan2(first_link_direction[1], first_link_direction[0])
first_link_angle_deg = np.degrees(first_link_angle % (2 * np.pi))
print(f"First link angle: {first_link_angle_deg:.1f}°")
print(f"Within limits: {80 <= first_link_angle_deg <= 100}")

print("\n=== Tests Complete ===")
