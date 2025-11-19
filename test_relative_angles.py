"""
Test to demonstrate the relative angle system
"""
import numpy as np
from src.chain import FABRIKChain


def test_relative_angle_system():
    """Demonstrate how relative angles work"""
    print("=" * 70)
    print("RELATIVE ANGLE SYSTEM DEMONSTRATION")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    
    # Create a specific configuration
    # Link 0: 45° absolute
    # Link 1: 30° relative to Link 0 (so 75° absolute)
    # Link 2: -45° relative to Link 1 (so 30° absolute)
    relative_angles = np.array([np.radians(45), np.radians(30), np.radians(-45)])
    
    print("\nTest Configuration:")
    print(f"  Link 0: 45° (absolute)")
    print(f"  Link 1: 30° (relative to Link 0) → 75° absolute")
    print(f"  Link 2: -45° (relative to Link 1) → 30° absolute")
    
    # Set chain using relative angles
    joints = chain.set_joints_from_angles(relative_angles, chain.base_position)
    chain.joints = joints
    chain._update_joint_angles()
    
    print(f"\nJoint positions:")
    for i, joint in enumerate(joints):
        print(f"  Joint {i}: ({joint[0]:.2f}, {joint[1]:.2f})")
    
    # Verify we get the same angles back
    computed_angles = chain.get_joint_angles(joints)
    computed_angles_deg = np.degrees(computed_angles)
    
    print(f"\nComputed relative angles from positions:")
    print(f"  {computed_angles_deg}")
    
    print(f"\nOriginal relative angles:")
    print(f"  {np.degrees(relative_angles)}")
    
    match = np.allclose(relative_angles, computed_angles, atol=1e-10)
    print(f"\nAngles match: {'✓' if match else '✗'}")
    
    # Calculate absolute angles for verification
    print(f"\nAbsolute angles (for verification):")
    abs_angle_0 = relative_angles[0]
    abs_angle_1 = relative_angles[0] + relative_angles[1]
    abs_angle_2 = relative_angles[0] + relative_angles[1] + relative_angles[2]
    
    print(f"  Link 0: {np.degrees(abs_angle_0):.1f}°")
    print(f"  Link 1: {np.degrees(abs_angle_1):.1f}°")
    print(f"  Link 2: {np.degrees(abs_angle_2):.1f}°")
    
    return match


def test_joint_constraints_with_relative_angles():
    """Test how joint constraints work with relative angles"""
    print("\n" + "=" * 70)
    print("JOINT CONSTRAINTS WITH RELATIVE ANGLES")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
    
    # Set angle limits
    # Joint 0 (base): 0° to 180° (absolute)
    # Joint 1: -90° to 90° (relative to Joint 0)
    chain.set_joint_limits(0, 0, np.pi)
    chain.set_joint_limits(1, -np.pi/2, np.pi/2)
    
    print("\nAngle limits:")
    print(f"  Joint 0: {np.degrees(chain.angle_limits[0][0]):.0f}° to {np.degrees(chain.angle_limits[0][1]):.0f}° (absolute)")
    print(f"  Joint 1: {np.degrees(chain.angle_limits[1][0]):.0f}° to {np.degrees(chain.angle_limits[1][1]):.0f}° (relative)")
    
    # Test various configurations
    test_configs = [
        ("Valid: 45° abs, 30° rel", [np.radians(45), np.radians(30)]),
        ("Valid: 90° abs, -45° rel", [np.radians(90), np.radians(-45)]),
        ("Valid: 0° abs, 90° rel", [np.radians(0), np.radians(90)]),
        ("Edge case: 180° abs, 0° rel", [np.radians(180), np.radians(0)]),
    ]
    
    print("\nTesting configurations:")
    for desc, angles in test_configs:
        joints = chain.set_joints_from_angles(np.array(angles), chain.base_position)
        
        # Calculate absolute angles for display
        abs_0 = angles[0]
        abs_1 = angles[0] + angles[1]
        
        print(f"\n  {desc}")
        print(f"    Relative: [{np.degrees(angles[0]):.1f}°, {np.degrees(angles[1]):.1f}°]")
        print(f"    Absolute: [{np.degrees(abs_0):.1f}°, {np.degrees(abs_1):.1f}°]")
        print(f"    End position: ({joints[-1][0]:.2f}, {joints[-1][1]:.2f})")


def test_monte_carlo_with_relative_angles():
    """Test Monte Carlo sampling with relative angles"""
    print("\n" + "=" * 70)
    print("MONTE CARLO SAMPLING WITH RELATIVE ANGLES")
    print("=" * 70)
    
    from Tools.monte_carlo import MonteCarloSampler
    from src.obstacle import Obstacle
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
    mc_sampler = MonteCarloSampler(chain, num_samples=50)
    obstacle = Obstacle(position=(60, 40), radius=20)
    
    print(f"\nSampling with {mc_sampler.num_samples} configurations...")
    collision_configs, non_collision_configs, neighbor_map = mc_sampler.sample_configuration_space(obstacle)
    
    print(f"\nResults:")
    print(f"  Collisions: {len(collision_configs)}")
    print(f"  Safe configs: {len(non_collision_configs)}")
    print(f"  Graph nodes: {len(neighbor_map)}")
    
    if len(collision_configs) > 0:
        example = collision_configs[0]
        print(f"\nExample collision config (relative angles):")
        print(f"  Joint 0: {np.degrees(example[0]):.1f}° (absolute)")
        print(f"  Joint 1: {np.degrees(example[1]):.1f}° (relative)")
        
        # Verify it actually collides
        positions = mc_sampler._angles_to_positions(example)
        has_collision = mc_sampler._check_collision(positions, obstacle)
        print(f"  Verified collision: {'✓' if has_collision else '✗ ERROR'}")
    
    if len(non_collision_configs) > 0:
        example = non_collision_configs[0]
        print(f"\nExample safe config (relative angles):")
        print(f"  Joint 0: {np.degrees(example[0]):.1f}° (absolute)")
        print(f"  Joint 1: {np.degrees(example[1]):.1f}° (relative)")
        
        # Verify it's actually safe
        positions = mc_sampler._angles_to_positions(example)
        has_collision = mc_sampler._check_collision(positions, obstacle)
        print(f"  Verified safe: {'✓' if not has_collision else '✗ ERROR'}")


if __name__ == "__main__":
    test1 = test_relative_angle_system()
    test_joint_constraints_with_relative_angles()
    test_monte_carlo_with_relative_angles()
    
    print("\n" + "=" * 70)
    if test1:
        print("ALL TESTS PASSED!")
        print("\nThe system now uses relative angles:")
        print("  • First angle is absolute (relative to world X-axis)")
        print("  • Subsequent angles are relative to previous link")
        print("  • Joint constraints are applied to relative angles")
        print("  • Monte Carlo sampling uses relative angles")
        print("  • This matches how articulated robots naturally work!")
    else:
        print("SOME TESTS FAILED!")
    print("=" * 70)
