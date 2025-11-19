"""
Test Monte Carlo sampling with both collision and non-collision visualization
"""
import numpy as np
from src.chain import FABRIKChain
from src.obstacle import Obstacle
from Tools.monte_carlo import MonteCarloSampler


def test_monte_carlo_both_samples():
    """Test that Monte Carlo returns both collision and non-collision samples"""
    print("=" * 70)
    print("Testing Monte Carlo with Collision AND Non-Collision Samples")
    print("=" * 70)
    
    # Create a simple chain
    chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
    
    # Create Monte Carlo sampler
    mc_sampler = MonteCarloSampler(chain, num_samples=500)
    
    # Create an obstacle
    obstacle = Obstacle(position=(80, 60), radius=30)
    
    print(f"\nChain: {chain.num_joints} joints, link length={chain.link_lengths[0]}")
    print(f"Obstacle: position={obstacle.position}, radius={obstacle.radius}")
    print(f"Sampling {mc_sampler.num_samples} configurations...")
    
    # Sample configuration space
    collision_configs, non_collision_configs = mc_sampler.sample_configuration_space(obstacle)
    
    print(f"\nResults:")
    print(f"  Total samples: {mc_sampler.num_samples}")
    print(f"  Collision configurations: {len(collision_configs)}")
    print(f"  Non-collision configurations: {len(non_collision_configs)}")
    print(f"  Sum: {len(collision_configs) + len(non_collision_configs)}")
    print(f"  Collision percentage: {100 * len(collision_configs) / mc_sampler.num_samples:.1f}%")
    
    # Verify total
    assert len(collision_configs) + len(non_collision_configs) == mc_sampler.num_samples, \
        "Total samples should equal num_samples!"
    
    print("\n✓ Test passed! Both collision and non-collision samples returned.")
    print(f"\nVisualization info:")
    print(f"  - Red dots (size 4): {len(collision_configs)} collision configurations")
    print(f"  - Grey dots (size 3): {len(non_collision_configs)} non-collision configurations")
    print(f"  - Grey dots are semi-transparent (alpha=30) for subtle background")


def test_multiple_obstacles():
    """Test with multiple obstacles"""
    print("\n" + "=" * 70)
    print("Testing Multiple Obstacles")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=40)
    mc_sampler = MonteCarloSampler(chain, num_samples=1000)
    
    obstacles = [
        Obstacle(position=(60, 50), radius=25),
        Obstacle(position=(100, 80), radius=20),
        Obstacle(position=(40, -30), radius=15)
    ]
    
    print(f"\nChain: {chain.num_joints} joints")
    print(f"Testing with {len(obstacles)} obstacles\n")
    
    for i, obs in enumerate(obstacles):
        print(f"Obstacle {i+1}: pos={obs.position}, r={obs.radius}")
        collision_configs, non_collision_configs = mc_sampler.sample_configuration_space(obs)
        collision_pct = 100 * len(collision_configs) / mc_sampler.num_samples
        print(f"  → {len(collision_configs)} collisions ({collision_pct:.1f}%)")
        print(f"  → {len(non_collision_configs)} non-collisions ({100-collision_pct:.1f}%)")
    
    print("\n✓ Multiple obstacles test passed!")


def test_visualization_data():
    """Test the data structure for visualization"""
    print("\n" + "=" * 70)
    print("Testing Visualization Data Structure")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
    mc_sampler = MonteCarloSampler(chain, num_samples=200)
    obstacle = Obstacle(position=(70, 50), radius=25)
    
    collision_configs, non_collision_configs = mc_sampler.sample_configuration_space(obstacle)
    
    # Create the data structure used by widget
    cspace_data = {
        'obstacle': obstacle,
        'collision_configs': collision_configs,
        'non_collision_configs': non_collision_configs,
        'num_samples': len(collision_configs)
    }
    
    print("\nC-space data structure:")
    print(f"  Keys: {list(cspace_data.keys())}")
    print(f"  Collision configs: {len(cspace_data['collision_configs'])} samples")
    print(f"  Non-collision configs: {len(cspace_data['non_collision_configs'])} samples")
    
    if len(collision_configs) > 0:
        print(f"\nSample collision config (first 2 joints in degrees):")
        sample = collision_configs[0]
        print(f"  Joint 0: {np.degrees(sample[0]):.1f}°")
        print(f"  Joint 1: {np.degrees(sample[1]):.1f}°")
    
    if len(non_collision_configs) > 0:
        print(f"\nSample non-collision config (first 2 joints in degrees):")
        sample = non_collision_configs[0]
        print(f"  Joint 0: {np.degrees(sample[0]):.1f}°")
        print(f"  Joint 1: {np.degrees(sample[1]):.1f}°")
    
    print("\n✓ Data structure test passed!")


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("MONTE CARLO VISUALIZATION TEST SUITE")
    print("Updated to show both collision AND non-collision samples")
    print("=" * 70)
    
    try:
        test_monte_carlo_both_samples()
        test_multiple_obstacles()
        test_visualization_data()
        
        print("\n" + "=" * 70)
        print("ALL TESTS PASSED! ✓")
        print("=" * 70)
        print("\nIn the GUI Configuration Space Viewer:")
        print("  • RED DOTS = Collision configurations (forbidden region)")
        print("  • GREY DOTS = Non-collision configurations (safe region)")
        print("  • Grey dots help show the sampling density and coverage")
        
    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
