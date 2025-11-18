import src.usermath as usermath
import numpy as np

def solve_ik_user(chain, target):
    #For a 4-link chain
    free_link = chain.num_joints - 2
    D = np.zeros(chain.num_joints-1)  # distances between joints
    alpha = np.zeros(chain.num_joints) # relavtive angles (180 - angle between links)

    #D0 = distance from base to target
    D[-1] = np.linalg.norm(target - chain.joints[0])

    #Keep the same pose for the last joint first
    currentJointAngle = chain.joint_angles[-1]
    D[0] = usermath.findLengthFromAngle(chain.link_lengths[len(chain.link_lengths)-1], 
                                        chain.link_lengths[len(chain.link_lengths)-2],
                                        np.pi - currentJointAngle)
    alpha[0] = currentJointAngle

    #Move on to 2nd joint from the end
    currentJointAngle = chain.joint_angles[-2]
    D[1] = usermath.findLengthFromAngle(chain.link_lengths[len(chain.link_lengths)-3], 
                                        D[0],
                                        np.pi - currentJointAngle)
    alpha[1] = currentJointAngle
    #Move on to 3rd joint from the end
    D[2] = usermath.findLengthFromAngle(chain.link_lengths[len(chain.link_lengths)-4], 
                                        D[1],
                                        np.pi - currentJointAngle)
    lower_lim = max(abs(chain.link_lengths[len(chain.link_lengths)-3] - D[1]),
                        D[0] - chain.link_lengths[0])
    
    upper_lim = min(chain.link_lengths[len(chain.link_lengths)-3] + D[-1],
                    D[0] + chain.link_lengths[0])
    if D[2] < lower_lim:
        D[2] = lower_lim
        currentJointAngle = np.pi - usermath.findAngleFromLength(chain.link_lengths[len(chain.link_lengths)-4],
                                                                D[1],
                                                                D[-2])
    elif D[2] > upper_lim:
        D[2] = upper_lim
        currentJointAngle = np.pi - usermath.findAngleFromLength(chain.link_lengths[len(chain.link_lengths)-4],
                                                                D[1],
                                                                D[-2])

    alpha[2] = currentJointAngle

    # Calculate angle for first joint
    alpha[3] = usermath.findAngleFromLength(chain.link_lengths[0],
                                            D[2],
                                            D[1])+usermath.findAngleBetweenVectors((1,0),target - chain.joints[0])

    