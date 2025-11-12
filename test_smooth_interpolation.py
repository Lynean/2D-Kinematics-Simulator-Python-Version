"""
Test smooth interpolation toggle
"""
import numpy as np
from src.chain import FABRIKChain

print("=== Testing Smooth Interpolation Toggle ===\n")

# Create a chain
chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)

print("Test 1: Smooth Interpolation Enabled (default)")
print(f"Enable smooth interpolation: {chain.enable_smooth_interpolation}")
print(f"Is interpolating: {chain.is_interpolating}")
print(f"Initial joints:\n{chain.joints}\n")

# Create a target pose
target_joints = np.array([[0, 0], [35, 35], [70, 70]])

# Start interpolation
chain.update_interpolation(poseA=chain.joints, poseB=target_joints)
print(f"After starting interpolation:")
print(f"Is interpolating: {chain.is_interpolating}")
print(f"Progress: {chain.interpolation_progress:.2f}\n")

# Simulate a few steps
for i in range(3):
    chain.update_interpolation()
    print(f"Step {i+1}: Progress = {chain.interpolation_progress:.2f}")

print(f"\nCurrent joints after 3 steps:\n{chain.joints}\n")

# Test 2: Disable smooth interpolation
print("\nTest 2: Smooth Interpolation Disabled")
chain2 = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
chain2.enable_smooth_interpolation = False

print(f"Enable smooth interpolation: {chain2.enable_smooth_interpolation}")
print(f"Initial joints:\n{chain2.joints}\n")

# Try to start interpolation (should be ignored when disabled)
target_joints2 = np.array([[0, 0], [35, 35], [70, 70]])
chain2.update_interpolation(poseA=chain2.joints, poseB=target_joints2)

print(f"After trying to start interpolation:")
print(f"Is interpolating: {chain2.is_interpolating}")
print("(Should remain False when smooth interpolation is disabled)\n")

# Test 3: Stop interpolation when disabling mid-way
print("\nTest 3: Disable Mid-Interpolation")
chain3 = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
chain3.update_interpolation(poseA=chain3.joints, poseB=target_joints)

print(f"Is interpolating: {chain3.is_interpolating}")
print(f"Progress: {chain3.interpolation_progress:.2f}")

# Disable smooth interpolation
chain3.enable_smooth_interpolation = False
chain3.is_interpolating = False  # Should be stopped by UI handler

print(f"\nAfter disabling:")
print(f"Enable smooth interpolation: {chain3.enable_smooth_interpolation}")
print(f"Is interpolating: {chain3.is_interpolating}")
print(f"Progress: {chain3.interpolation_progress:.2f}")

print("\n=== Tests Complete ===")
