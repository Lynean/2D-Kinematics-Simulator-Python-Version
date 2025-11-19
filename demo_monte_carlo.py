"""
Quick Demo: Monte Carlo C-Space Obstacle Mapping
Shows how workspace obstacles map to configuration space
"""
from src.chain import FABRIKChain
from src.obstacle import Obstacle
from Tools.monte_carlo import MonteCarloSampler
import numpy as np


def demo_cspace_mapping():
    """
    Demonstrates how a workspace obstacle creates a forbidden region
    in configuration space using Monte Carlo sampling.
    """
    print("\n" + "="*70)
    print("MONTE CARLO C-SPACE OBSTACLE MAPPING DEMO")
    print("="*70)
    
    # Create a 3-joint robot arm
    print("\n1. Creating robot arm...")
    chain = FABRIKChain(
        base_position=(0, 0),
        num_joints=3,
        link_length=50
    )
    print(f"   ✓ Robot: {chain.num_joints} joints, {chain.link_lengths[0]}mm links")
    
    # Create an obstacle in the workspace
    print("\n2. Placing obstacle in workspace...")
    obstacle = Obstacle(
        position=(80, 60),  # Workspace coordinates
        radius=30           # Obstacle size
    )
    print(f"   ✓ Obstacle at {obstacle.position}, radius={obstacle.radius}mm")
    
    # Initialize Monte Carlo sampler
    print("\n3. Initializing Monte Carlo sampler...")
    mc_sampler = MonteCarloSampler(
        chain=chain,
        num_samples=1000  # Test 1000 random configurations
    )
    print(f"   ✓ Sampler ready with {mc_sampler.num_samples} samples")
    
    # Sample configuration space
    print("\n4. Sampling configuration space...")
    print("   (Testing random joint angles to find collisions...)")
    
    collision_configs = mc_sampler.sample_configuration_space(obstacle)
    
    collision_pct = 100 * len(collision_configs) / mc_sampler.num_samples
    print(f"\n   ✓ Found {len(collision_configs)} collision configurations")
    print(f"   ✓ Collision rate: {collision_pct:.1f}%")
    
    # Show some example collision configurations
    if len(collision_configs) > 0:
        print("\n5. Example collision configurations:")
        print("   (Joint angles that cause robot to hit obstacle)\n")
        
        for i, config in enumerate(collision_configs[:5]):
            config_deg = np.degrees(config)
            print(f"   Config {i+1}:")
            print(f"      Joint 0: {config_deg[0]:6.1f}°")
            print(f"      Joint 1: {config_deg[1]:6.1f}°")
            if len(config_deg) > 2:
                print(f"      Joint 2: {config_deg[2]:6.1f}°")
            print()
    
    # Show 2D projection
    print("6. Configuration space analysis:")
    region_2d = mc_sampler.get_obstacle_region_2d(obstacle)
    
    if region_2d and len(region_2d) > 0:
        angles = np.array(region_2d)
        joint0_deg = np.degrees(angles[:, 0])
        joint1_deg = np.degrees(angles[:, 1])
        
        print(f"   ✓ 2D C-space projection (Joint 0 vs Joint 1):")
        print(f"      Joint 0 forbidden range: {joint0_deg.min():.1f}° to {joint0_deg.max():.1f}°")
        print(f"      Joint 1 forbidden range: {joint1_deg.min():.1f}° to {joint1_deg.max():.1f}°")
        print(f"      Forbidden region points: {len(region_2d)}")
    
    print("\n" + "="*70)
    print("INTERPRETATION")
    print("="*70)
    print("""
    The Monte Carlo sampler tested 1000 random robot configurations.
    
    For each configuration:
    - Random joint angles were generated within constraints
    - Forward kinematics computed link positions in workspace
    - Collision detection checked if links intersect obstacle
    
    The collision configurations form a "forbidden region" in
    configuration space. These are joint angle combinations that
    CANNOT be used because they cause the robot to hit the obstacle.
    
    In the GUI:
    - Workspace view shows obstacles as circles
    - Configuration Space Viewer shows forbidden regions as red points
    - This helps understand which joint movements to avoid!
    """)
    print("="*70 + "\n")


if __name__ == "__main__":
    demo_cspace_mapping()
    
    print("TIP: Run the main GUI (python main.py) to see this in action!")
    print("     1. Click 'Add Obstacle'")
    print("     2. Click in workspace to place obstacle")
    print("     3. Open 'Configuration Space Viewer'")
    print("     4. See red forbidden regions in 2D C-space plot!\n")
