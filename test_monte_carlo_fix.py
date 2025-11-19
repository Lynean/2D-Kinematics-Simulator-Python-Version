"""
Test to verify Monte Carlo collision detection fix
"""
import numpy as np
from src.chain import FABRIKChain
from src.obstacle import Obstacle
from Tools.monte_carlo import MonteCarloSampler


def test_angles_to_positions():
    """Test that _angles_to_positions matches chain.set_joints_from_angles"""
    print("=" * 70)
    print("TESTING ANGLES TO POSITIONS CONSISTENCY")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    mc_sampler = MonteCarloSampler(chain, num_samples=10)
    
    # Test with known angles
    test_angles = [
        np.array([0, np.pi/4, np.pi/2]),  # 0°, 45°, 90°
        np.array([np.pi/6, np.pi/3, np.pi/2]),  # 30°, 60°, 90°
        np.array([0, 0, 0]),  # All horizontal
    ]
    
    all_match = True
    for angles in test_angles:
        # Get positions from Monte Carlo method
        mc_positions = mc_sampler._angles_to_positions(angles)
        
        # Get positions from chain method
        chain_positions = chain.set_joints_from_angles(angles, chain.base_position)
        
        # Compare
        match = np.allclose(mc_positions, chain_positions, atol=1e-10)
        
        angles_deg = np.degrees(angles)
        print(f"\nTest angles: {angles_deg}")
        print(f"  Monte Carlo positions:\n{mc_positions}")
        print(f"  Chain positions:\n{chain_positions}")
        print(f"  Match: {'✓' if match else '✗'}")
        
        if not match:
            all_match = False
            print(f"  Difference:\n{mc_positions - chain_positions}")
    
    if all_match:
        print("\n✓ All positions match! Monte Carlo uses correct angle interpretation.")
    else:
        print("\n✗ Positions don't match! There's still an issue.")
    
    return all_match


def test_collision_consistency():
    """Test that clicking on a datapoint shows correct collision status"""
    print("\n" + "=" * 70)
    print("TESTING COLLISION DETECTION CONSISTENCY")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
    mc_sampler = MonteCarloSampler(chain, num_samples=100)
    
    # Create obstacle
    obstacle = Obstacle(position=(60, 40), radius=20)
    
    # Sample configuration space
    collision_configs, non_collision_configs, _ = mc_sampler.sample_configuration_space(obstacle)
    
    print(f"\nObstacle: position={obstacle.position}, radius={obstacle.radius}")
    print(f"Collision configs: {len(collision_configs)}")
    print(f"Non-collision configs: {len(non_collision_configs)}")
    
    # Test a few collision configs
    print("\nVerifying collision configs are actually in collision:")
    errors = 0
    for i, config in enumerate(collision_configs[:5]):  # Test first 5
        positions = mc_sampler._angles_to_positions(config)
        has_collision = mc_sampler._check_collision(positions, obstacle)
        
        status = "✓" if has_collision else "✗ ERROR"
        if not has_collision:
            errors += 1
        
        config_deg = np.degrees(config)
        print(f"  Config {i}: {config_deg} -> {status}")
    
    # Test a few non-collision configs
    print("\nVerifying non-collision configs are actually safe:")
    for i, config in enumerate(non_collision_configs[:5]):  # Test first 5
        positions = mc_sampler._angles_to_positions(config)
        has_collision = mc_sampler._check_collision(positions, obstacle)
        
        status = "✓" if not has_collision else "✗ ERROR"
        if has_collision:
            errors += 1
        
        config_deg = np.degrees(config)
        print(f"  Config {i}: {config_deg} -> {status}")
    
    if errors == 0:
        print("\n✓ All collision detections are consistent!")
    else:
        print(f"\n✗ Found {errors} inconsistencies!")
    
    return errors == 0


if __name__ == "__main__":
    test1_pass = test_angles_to_positions()
    test2_pass = test_collision_consistency()
    
    print("\n" + "=" * 70)
    if test1_pass and test2_pass:
        print("ALL TESTS PASSED!")
    else:
        print("SOME TESTS FAILED!")
    print("=" * 70)
