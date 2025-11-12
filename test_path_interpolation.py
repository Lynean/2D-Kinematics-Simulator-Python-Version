"""
Test path interpolation toggle
"""
import numpy as np
from src.path import Path

print("=== Testing Path Interpolation Toggle ===\n")

# Create a path
path = Path(step_size=10.0)

# Add some points
points = [
    [0, 0],
    [100, 0],
    [100, 100],
    [0, 100]
]

for point in points:
    path.add_point(point)

print("Test 1: With Interpolation Enabled (default)")
print(f"Enable interpolation: {path.enable_interpolation}")
print(f"Step size: {path.step_size}")
print(f"Raw points: {len(path.raw_points)}")
print(f"Interpolated points: {len(path.interpolated_points)}")
print(f"First few interpolated: {path.interpolated_points[:5]}\n")

# Disable interpolation
print("Test 2: With Interpolation Disabled")
path.enable_interpolation = False
path.interpolate()
print(f"Enable interpolation: {path.enable_interpolation}")
print(f"Raw points: {len(path.raw_points)}")
print(f"Interpolated points: {len(path.interpolated_points)}")
print(f"Should match raw points: {path.interpolated_points == path.raw_points}")
print(f"Interpolated points: {path.interpolated_points}\n")

# Re-enable interpolation
print("Test 3: Re-enable Interpolation")
path.enable_interpolation = True
path.interpolate()
print(f"Enable interpolation: {path.enable_interpolation}")
print(f"Interpolated points: {len(path.interpolated_points)}")
print(f"Should be > raw points: {len(path.interpolated_points) > len(path.raw_points)}\n")

# Test with step size = 1
print("Test 4: Step Size = 1")
path.set_step_size(1.0)
print(f"Step size: {path.step_size}")
print(f"Interpolated points: {len(path.interpolated_points)}")
print(f"(Should have many points with step size 1)\n")

# Test A* path
print("Test 5: A* Path with Interpolation Disabled")
path2 = Path(step_size=10.0)
path2.enable_interpolation = False
astar_waypoints = [[0, 0], [50, 50], [100, 100]]
path2.set_astar_path(astar_waypoints)
print(f"A* waypoints: {len(path2.astar_waypoints)}")
print(f"Interpolated points: {len(path2.interpolated_points)}")
print(f"Should match waypoints: {path2.interpolated_points == path2.astar_waypoints}")

print("\n=== Tests Complete ===")
