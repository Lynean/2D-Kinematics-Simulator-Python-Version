"""
Debug base joint constraint application
"""
import numpy as np
from src.chain import FABRIKChain

print("=== Debug Base Joint Constraint ===\n")

chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
chain.set_joint_limits(0, np.radians(45), np.radians(135))

print(f"Base limits: {np.degrees(chain.angle_limits[0][0]):.1f}° to {np.degrees(chain.angle_limits[0][1]):.1f}°")
print(f"Initial first link angle: {np.degrees(np.arctan2(chain.joints[1][1] - chain.joints[0][1], chain.joints[1][0] - chain.joints[0][0])):.1f}°\n")

# Target at 0 degrees (horizontal right) - should be clamped to 45°
target = np.array([100, 0])
print(f"Target: {target}")
print(f"Distance: {np.linalg.norm(target - chain.base_position):.1f}")
print(f"Total length: {sum(chain.link_lengths):.1f}")
print(f"Reachable: {np.linalg.norm(target - chain.base_position) <= sum(chain.link_lengths)}")
print(f"Tolerance: {chain.tolerance}")
print(f"Initial end-effector position: {chain.joints[-1]}")
print(f"Initial diff: {np.linalg.norm(chain.joints[-1] - target):.2f}\n")

result = chain.solve(target)

print(f"\nAfter solving:")
print(f"Joints:\n{chain.joints}")

first_link = chain.joints[1] - chain.joints[0]
angle = np.arctan2(first_link[1], first_link[0])
angle_deg = np.degrees(angle % (2 * np.pi))

print(f"\nFirst link direction: {first_link}")
print(f"First link angle: {angle_deg:.1f}°")
print(f"Expected: 45° (clamped from 0°)")
