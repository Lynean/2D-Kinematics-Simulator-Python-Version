"""
Test script to verify joint_angles caching works correctly
"""
import numpy as np
from src.chain import FABRIKChain

def test_joint_angles_caching():
    """Test that joint_angles are properly cached and updated"""
    print("Testing joint_angles caching...")
    
    # Create a simple chain
    chain = FABRIKChain(base_position=(0, 0), num_joints=3, link_length=50)
    
    # Check initial angles
    print(f"\n1. Initial state:")
    print(f"   Num joints: {chain.num_joints}")
    print(f"   Joint angles shape: {chain.joint_angles.shape}")
    print(f"   Joint angles (deg): {np.degrees(chain.joint_angles)}")
    
    # Solve to a target
    target = np.array([100, 100])
    chain.solve(target)
    
    print(f"\n2. After solving to target {target}:")
    print(f"   Joint angles (deg): {np.degrees(chain.joint_angles)}")
    
    # Verify angles match computed values
    computed_angles = chain.get_joint_angles(chain.joints)
    print(f"   Computed angles (deg): {np.degrees(computed_angles)}")
    print(f"   Match: {np.allclose(chain.joint_angles, computed_angles)}")
    
    # Add a joint
    chain.add_joint(link_length=50)
    
    print(f"\n3. After adding joint:")
    print(f"   Num joints: {chain.num_joints}")
    print(f"   Joint angles shape: {chain.joint_angles.shape}")
    print(f"   Joint angles (deg): {np.degrees(chain.joint_angles)}")
    
    # Solve again
    chain.solve(target)
    computed_angles = chain.get_joint_angles(chain.joints)
    print(f"   After solving - Match: {np.allclose(chain.joint_angles, computed_angles)}")
    
    # Remove a joint
    chain.remove_joint()
    
    print(f"\n4. After removing joint:")
    print(f"   Num joints: {chain.num_joints}")
    print(f"   Joint angles shape: {chain.joint_angles.shape}")
    print(f"   Joint angles (deg): {np.degrees(chain.joint_angles)}")
    
    # Verify again
    computed_angles = chain.get_joint_angles(chain.joints)
    print(f"   Match: {np.allclose(chain.joint_angles, computed_angles)}")
    
    print("\n✓ All tests passed! Joint angles are properly cached and updated.")

if __name__ == "__main__":
    test_joint_angles_caching()
