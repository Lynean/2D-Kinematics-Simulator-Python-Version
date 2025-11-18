"""
Simple direct test for ik_user.py
Shows step-by-step what the solver computes
"""
import numpy as np
from src.chain import FABRIKChain
from src.ik_user import solve_ik_user


def test_solver_output():
    """Test what the solver actually outputs"""
    print("=" * 70)
    print("SIMPLE IK_USER SOLVER TEST")
    print("=" * 70)
    
    # Create a 4-joint chain
    chain = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    
    print("\nInitial Chain Configuration:")
    print(f"  Base: {chain.joints[0]}")
    print(f"  Link lengths: {chain.link_lengths}")
    print(f"  Total length: {sum(chain.link_lengths)}")
    print(f"  Joint angles (cached): {np.degrees(chain.joint_angles)}°")
    
    for i in range(chain.num_joints):
        print(f"  Joint {i}: {chain.joints[i]}")
    
    # Set target
    target = np.array([100, 100])
    distance_to_target = np.linalg.norm(target - chain.joints[0])
    
    print(f"\nTarget: {target}")
    print(f"Distance from base to target: {distance_to_target:.2f}")
    print(f"Reachable: {distance_to_target <= sum(chain.link_lengths)}")
    
    # Call the solver
    print("\nCalling solve_ik_user...")
    try:
        alpha = solve_ik_user(chain, target)
        
        print(f"\nSolver Output (alpha array):")
        print(f"  Shape: {alpha.shape}")
        print(f"  Type: {type(alpha)}")
        print(f"\nAlpha values:")
        for i in range(len(alpha)):
            print(f"  alpha[{i}] = {alpha[i]:.4f} rad = {np.degrees(alpha[i]):.2f}°")
        
        print(f"\nInterpretation:")
        print(f"  alpha[0] = angle between last two links (joint 2 angle)")
        print(f"  alpha[1] = angle between links 1-2 (joint 1 angle)")  
        print(f"  alpha[2] = angle between links 0-1 (joint 0 relative angle)")
        print(f"  alpha[3] = absolute angle of first link from horizontal")
        
        print(f"\n✓ Solver executed successfully!")
        print(f"\nNote: The solver returns relative angles that need to be")
        print(f"      converted to joint positions using forward kinematics.")
        
        return alpha
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_with_fabrik_comparison():
    """Compare user IK with FABRIK"""
    print("\n" + "=" * 70)
    print("COMPARISON: IK_USER vs FABRIK")
    print("=" * 70)
    
    # Test 1: User IK
    chain1 = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    chain1.ik_method = 'FABRIK'
    target = np.array([100, 80])
    
    print("\n1. FABRIK Solution:")
    chain1.solve(target)
    print(f"   End effector: {chain1.joints[-1]}")
    print(f"   Distance to target: {np.linalg.norm(chain1.joints[-1] - target):.2f}")
    print(f"   Joint angles: {np.degrees(chain1.joint_angles)}°")
    
    # Test 2: User IK
    chain2 = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    
    print("\n2. User IK Solution:")
    alpha = solve_ik_user(chain2, target)
    print(f"   Alpha values (deg): {np.degrees(alpha)}°")
    print(f"\n   Note: User IK returns angles, not positions.")
    print(f"   Would need additional code to apply these angles to the chain.")


def main():
    test_solver_output()
    test_with_fabrik_comparison()
    
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print("""
The ik_user.py solver:
- Takes a chain and target position as input
- Returns an array of 4 angles (alpha)
- These represent relative/absolute joint angles
- The angles need to be converted to joint positions

The solver appears to use an analytical approach based on:
- Law of cosines (via usermath functions)
- Geometric constraints
- Distance calculations between virtual joints

To fully integrate this solver, you would need to:
1. Convert the alpha angles to absolute link angles
2. Use forward kinematics to position each joint
3. Apply the results to chain.joints
4. Update chain._update_joint_angles()
    """)


if __name__ == "__main__":
    main()
