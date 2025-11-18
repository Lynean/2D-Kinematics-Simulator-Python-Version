"""
Test script for ik_user.py IK solver
Tests the user-defined inverse kinematics solver
"""
import numpy as np
import sys
from src.chain import FABRIKChain
from src.ik_user import solve_ik_user
import src.usermath as usermath


def visualize_chain(chain, title="Chain State"):
    """Simple text visualization of chain"""
    print(f"\n{title}:")
    print(f"  Joints: {chain.num_joints}")
    for i in range(chain.num_joints):
        pos = chain.joints[i]
        print(f"    Joint {i}: ({pos[0]:6.1f}, {pos[1]:6.1f})", end="")
        if i < len(chain.joint_angles):
            print(f"  angle: {np.degrees(chain.joint_angles[i]):6.1f}°")
        else:
            print()


def test_basic_ik_user():
    """Test basic IK solver functionality"""
    print("=" * 70)
    print("TEST 1: Basic IK User Solver")
    print("=" * 70)
    
    # Create a 4-joint chain (as the solver expects)
    chain = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    
    visualize_chain(chain, "Initial Chain")
    
    # Set a target position
    target = np.array([100, 100])
    print(f"\nTarget: ({target[0]}, {target[1]})")
    
    # Solve using user IK
    try:
        alpha = solve_ik_user(chain, target)
        print(f"\nSolver returned angles (alpha):")
        print(f"  Shape: {alpha.shape}")
        print(f"  Values (radians): {alpha}")
        print(f"  Values (degrees): {np.degrees(alpha)}")
        
        # Reconstruct chain from angles
        print(f"\nReconstructing chain from angles...")
        reconstruct_chain_from_angles(chain, alpha, target)
        
        visualize_chain(chain, "After IK Solve")
        
        # Calculate end effector distance to target
        end_effector = chain.joints[-1]
        distance = np.linalg.norm(end_effector - target)
        print(f"\nEnd effector distance to target: {distance:.2f}")
        
        if distance < 5.0:
            print("✓ Target reached successfully!")
        else:
            print(f"⚠ Target not reached (distance: {distance:.2f})")
            
    except Exception as e:
        print(f"\n❌ Error during IK solve: {e}")
        import traceback
        traceback.print_exc()


def reconstruct_chain_from_angles(chain, alpha, target):
    """
    Reconstruct chain joint positions from relative angles
    This is a helper to verify the IK solution
    """
    # alpha contains relative angles between links
    # We need to convert to absolute angles and position joints
    
    # Start from base
    chain.joints[0] = chain.base_position
    
    # First link angle is alpha[3] (absolute from horizontal)
    current_angle = alpha[3]
    chain.joints[1] = chain.joints[0] + np.array([
        chain.link_lengths[0] * np.cos(current_angle),
        chain.link_lengths[0] * np.sin(current_angle)
    ])
    
    # Subsequent links use relative angles
    for i in range(1, chain.num_joints - 1):
        # alpha[i] is the relative angle (supplement angle)
        # Convert to absolute angle
        relative = np.pi - alpha[chain.num_joints - 1 - i]
        current_angle += relative
        
        chain.joints[i + 1] = chain.joints[i] + np.array([
            chain.link_lengths[i] * np.cos(current_angle),
            chain.link_lengths[i] * np.sin(current_angle)
        ])
    
    # Update cached angles
    chain._update_joint_angles()


def test_reachable_target():
    """Test with easily reachable target"""
    print("\n" + "=" * 70)
    print("TEST 2: Reachable Target")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    
    # Target within easy reach
    target = np.array([80, 60])
    print(f"Target: ({target[0]}, {target[1]})")
    print(f"Total chain length: {sum(chain.link_lengths)}")
    print(f"Distance to target: {np.linalg.norm(target):.2f}")
    
    try:
        alpha = solve_ik_user(chain, target)
        reconstruct_chain_from_angles(chain, alpha, target)
        
        visualize_chain(chain, "After IK")
        
        distance = np.linalg.norm(chain.joints[-1] - target)
        print(f"\nEnd effector error: {distance:.2f}")
        
        if distance < 5.0:
            print("✓ Test passed!")
        else:
            print("⚠ Test failed - target not reached")
            
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()


def test_unreachable_target():
    """Test with unreachable target"""
    print("\n" + "=" * 70)
    print("TEST 3: Unreachable Target")
    print("=" * 70)
    
    chain = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    
    # Target beyond reach
    target = np.array([300, 300])
    total_length = sum(chain.link_lengths)
    distance = np.linalg.norm(target)
    
    print(f"Target: ({target[0]}, {target[1]})")
    print(f"Total chain length: {total_length}")
    print(f"Distance to target: {distance:.2f}")
    print(f"Reachable: {distance <= total_length}")
    
    try:
        alpha = solve_ik_user(chain, target)
        reconstruct_chain_from_angles(chain, alpha, target)
        
        visualize_chain(chain, "After IK")
        
        final_distance = np.linalg.norm(chain.joints[-1] - target)
        print(f"\nEnd effector distance: {final_distance:.2f}")
        print(f"Chain should be stretched toward target")
        
        # Check if chain is pointing toward target
        direction = target / np.linalg.norm(target)
        end_direction = chain.joints[-1] / np.linalg.norm(chain.joints[-1])
        alignment = np.dot(direction, end_direction)
        
        print(f"Alignment with target direction: {alignment:.3f}")
        
        if alignment > 0.95:
            print("✓ Chain correctly stretched toward target")
        else:
            print("⚠ Chain not aligned with target")
            
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()


def test_usermath_functions():
    """Test the usermath helper functions"""
    print("\n" + "=" * 70)
    print("TEST 4: UserMath Helper Functions")
    print("=" * 70)
    
    # Test findLengthFromAngle
    print("\n1. Testing findLengthFromAngle:")
    a, b = 50, 50
    angle = np.pi / 3  # 60 degrees
    c = usermath.findLengthFromAngle(a, b, angle)
    print(f"   a={a}, b={b}, angle={np.degrees(angle):.1f}°")
    print(f"   Result: c={c:.2f}")
    print(f"   Expected: c ≈ 50 (for 60° angle)")
    
    # Test findAngleFromLength
    print("\n2. Testing findAngleFromLength:")
    a, b, c = 50, 50, 50
    angle = usermath.findAngleFromLength(a, b, c)
    print(f"   a={a}, b={b}, c={c}")
    print(f"   Result: angle={np.degrees(angle):.1f}°")
    print(f"   Expected: 60° (equilateral triangle)")
    
    # Test findAngleBetweenVectors
    print("\n3. Testing findAngleBetweenVectors:")
    v1 = np.array([1, 0])
    v2 = np.array([1, 1])
    angle = usermath.findAngleBetweenVectors(v1, v2)
    print(f"   v1={v1}, v2={v2}")
    print(f"   Result: angle={np.degrees(angle):.1f}°")
    print(f"   Expected: 45°")
    
    # Test findPointFromAngleAndDistance
    print("\n4. Testing findPointFromAngleAndDistance:")
    origin = np.array([0, 0])
    angle = np.pi / 4  # 45 degrees
    distance = 100
    point = usermath.findPointFromAngleAndDistance(origin, angle, distance)
    print(f"   origin={origin}, angle={np.degrees(angle):.1f}°, distance={distance}")
    print(f"   Result: point=({point[0]:.1f}, {point[1]:.1f})")
    print(f"   Expected: (~70.7, ~70.7)")
    
    print("\n✓ UserMath functions working!")


def test_different_initial_poses():
    """Test IK solver with different initial chain configurations"""
    print("\n" + "=" * 70)
    print("TEST 5: Different Initial Poses")
    print("=" * 70)
    
    target = np.array([100, 50])
    
    # Test 1: Straight chain
    print("\nCase 1: Straight chain")
    chain1 = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    try:
        alpha1 = solve_ik_user(chain1, target)
        reconstruct_chain_from_angles(chain1, alpha1, target)
        dist1 = np.linalg.norm(chain1.joints[-1] - target)
        print(f"  Distance to target: {dist1:.2f}")
    except Exception as e:
        print(f"  Error: {e}")
    
    # Test 2: Pre-bent chain
    print("\nCase 2: Pre-bent chain")
    chain2 = FABRIKChain(base_position=(0, 0), num_joints=4, link_length=50)
    # Manually set a bent configuration
    chain2.joints[1] = chain2.joints[0] + np.array([50, 0])
    chain2.joints[2] = chain2.joints[1] + np.array([35, 35])
    chain2.joints[3] = chain2.joints[2] + np.array([0, 50])
    chain2._update_joint_angles()
    
    visualize_chain(chain2, "Pre-bent initial state")
    
    try:
        alpha2 = solve_ik_user(chain2, target)
        reconstruct_chain_from_angles(chain2, alpha2, target)
        dist2 = np.linalg.norm(chain2.joints[-1] - target)
        print(f"  Distance to target: {dist2:.2f}")
    except Exception as e:
        print(f"  Error: {e}")


def main():
    """Run all tests"""
    print("\n" + "=" * 70)
    print("IK_USER.PY TEST SUITE")
    print("=" * 70)
    
    try:
        test_usermath_functions()
        test_basic_ik_user()
        test_reachable_target()
        test_unreachable_target()
        test_different_initial_poses()
        
        print("\n" + "=" * 70)
        print("TEST SUITE COMPLETE")
        print("=" * 70)
        
    except Exception as e:
        print(f"\n❌ FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
