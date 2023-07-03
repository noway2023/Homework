import numpy as np
from scipy.spatial.transform import Rotation as R
import torch


def JacobianSolver(joint_name, joint_parent, path, path_name, joint_positions, joint_orientations, target_pose):
    
    return joint_positions, joint_orientations

def CCDSolver(joint_name, joint_parent, path, path_name, joint_positions, joint_orientations, target_pose):
    current_pose = joint_positions[path[-1]]
    tolerent = 1e-2
    itertime = 1
    maxiter = 100
    jointcount = len(path)
    currentjoint = jointcount - 1
    while np.sqrt(np.sum((current_pose - target_pose) ** 2)) > tolerent and itertime <= maxiter:
        currentjoint = currentjoint % jointcount
        if currentjoint == jointcount - 1:
            currentjoint -= 1
            continue
        # offset = [(joint_positions[path[i+1]] - joint_positions[path[i]]) for i in range(currentjoint, jointcount - 1)]
        offsetall =[(joint_positions[i]- joint_positions[joint_parent[i]])for i in range(1, len(joint_parent))]
        offsetall = [0] + offsetall
        vector1 = joint_positions[path[currentjoint]] - target_pose
        vector2 = joint_positions[path[currentjoint]] - joint_positions[path[-1]]
        axis = np.cross(vector2, vector1)
        axis = axis / np.linalg.norm(axis)
        angle = np.arccos(vector2.dot(vector1)/(np.linalg.norm(vector2) * np.linalg.norm(vector1)))
        rotvec = axis * angle
        rotation = R.from_rotvec(rotvec).as_matrix()
        
        old_rotation = R.from_quat(joint_orientations[path[currentjoint]]).as_matrix()
        new_rotation = R.from_matrix(rotation @ old_rotation).as_quat()
        joint_orientations[path[currentjoint]] = new_rotation
        # #only change the joint in path other joint donot transform
        # for i in range(currentjoint+1, jointcount):
            # joint_orientations[path[i]] =  R.from_matrix(rotation @ R.from_quat(joint_orientations[path[i]]).as_matrix()).as_quat()
            # joint_positions[path[i]] = joint_positions[path[i-1]] + rotation @ offset[i - currentjoint - 1].T
        
        current_index = path[currentjoint]
        stack = [current_index]
        while(stack != []): 
            stack.pop()
            child_joints = [i for i, x in enumerate(joint_parent) if x == current_index]
            for j in child_joints:
                if j not in stack:
                    stack.append(j)
                    joint_orientations[j] =  R.from_matrix( rotation @ R.from_quat(joint_orientations[j]).as_matrix()).as_quat()
                    joint_positions[j] = joint_positions[current_index] + rotation @ offsetall[j].T

            if stack:
                current_index = stack[-1]

        currentjoint -= 1
        itertime += 1
        
    print('iter times:', itertime)
    
    return joint_positions, joint_orientations

def part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose):
    """
    完成函数，计算逆运动学
    输入: 
        meta_data: 为了方便，将一些固定信息进行了打包，见上面的meta_data类
        joint_positions: 当前的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 当前的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
        target_pose: 目标位置，是一个numpy数组，shape为(3,)
    输出:
        经过IK后的姿态
        joint_positions: 计算得到的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 计算得到的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
    """
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()
    
    joint_parent = meta_data.joint_parent
    joint_name = meta_data.joint_name
    # CCD solver
    joint_positions, joint_orientations = CCDSolver(joint_name, joint_parent, path, path_name, joint_positions, joint_orientations, target_pose)
    # Jacabian solver
    
    
    
    return joint_positions, joint_orientations

def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    """
    输入lWrist相对于RootJoint前进方向的xz偏移，以及目标高度，IK以外的部分与bvh一致
    """
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()
    
    joint_parent = meta_data.joint_parent
    joint_name = meta_data.joint_name
    
    
    return joint_positions, joint_orientations

def bonus_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    """
    输入左手和右手的目标位置，固定左脚，完成函数，计算逆运动学
    """
    
    return joint_positions, joint_orientations
