import src.usermath as usermath
import numpy as np

def solve_2_joint_ik(chain, target, joint_index):
    #Treat the chain as a 2-joint chain at the "joint_index" with a fixed base
    d1 = usermath.findDistanceBetweenPoints(chain.joints[joint_index], 
                                            chain.base_position)
    d2 = usermath.findDistanceBetweenPoints(target, 
                                            chain.joints[joint_index])
    desired_dist = usermath.findDistanceBetweenPoints(chain.base_position, target)

    alpha = usermath.findAngleFromLength(d1, d2, desired_dist)
    theta2 = np.pi - alpha
    theta1 = usermath.findAngleBetweenVectors((target - chain.base_position), (1,0))
    return theta1,theta2

