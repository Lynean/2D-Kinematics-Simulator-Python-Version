"""
Simple test without constraints to verify FABRIK works
"""
import numpy as np
from src.chain import FABRIKChain

# Create a simple 3-joint chain
chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)

print("Testing FABRIK without constraints\n")
print(f"Initial positions:\n{chain.joints}\n")
print(f"Is interpolating: {chain.is_interpolating}")
print(f"End effector index: {chain.end_effector_index}\n")

target = np.array([60, 40])  # Reachable target
print(f"Target: {target}\n")

# Check reachability
total_length = sum(chain.link_lengths)
distance_to_target = np.linalg.norm(target - chain.base_position)
print(f"Total chain length: {total_length:.2f}")
print(f"Distance to target: {distance_to_target:.2f}")
print(f"Reachable: {distance_to_target <= total_length}\n")

# Temporarily disable constraints by setting all to full rotation
for i in range(chain.num_joints):
    chain.angle_limits[i] = (0, 2 * np.pi)

result = chain.solve(target)
print(f"Result: {'Converged' if result else 'Max iterations reached'}")
print(f"Iterations: {chain.current_iterations}")
print(f"\nFinal positions:\n{chain.joints}")

# Calculate error
end_effector = chain.joints[-1]
error = np.linalg.norm(end_effector - target)
print(f"\nError: {error:.2f}")
