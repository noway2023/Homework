import numpy as np
from scipy.spatial.transform import Rotation as R

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data

class TreeNode:
    def __init__(self, name):
        self.name = name
        self.children = []
        self.parent = []
        self.info = []
        
    def add_children(self, node):
        self.children.append(node)
        
    def get_children(self):
        return self.children
    
    def add_information(self, value):
        self.info.append(value)
        
    def show_info(self):
        print(self.info)
    
    def get_info(self):
        return self.info
        
    def get_parent(self):
        return self.parent
    
    def add_parent(self, node):
        self.parent.append(node)
        
    def get_name(self):
        return self.name

def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    joint_name = []
    joint_parent = []
    joint_offset = []
    lines = []
    startflag = False
    with open(bvh_file_path, 'r') as f:
        for line in f:   
            if not startflag:
                if 'HIERARCHY' in line:
                    startflag = True
                continue
            
            if 'MOTION' in line:
                break
            lines.append(line.strip())

    stack = []
    joint_dic = {}
    for line in lines:
        if '}' not in line: 
            stack.append(line)
            line = line.split()
            if 'ROOT' in line:
                joint_name.append(line[1])
                joint_dic[joint_name[-1]] = TreeNode(joint_name[-1])
            if 'JOINT' in line:
                joint_name.append(line[1])
                joint_dic[joint_name[-1]] = TreeNode(joint_name[-1])
            if 'End' in line:
                joint_name.append(joint_name[-1] + '_end')
                joint_dic[joint_name[-1]] = TreeNode(joint_name[-1])
            
        else:
            info = []
            while('{' not in stack[-1]):
                info.append(stack[-1])
                stack.pop()
            stack.pop()
            if 'End' in stack[-1]:
                joint_dic[joint_name[-1]].add_information(info)                
            else:
                joint_dic[stack[-1].split()[1]].add_information(info)
                if 'End' in info[0]:
                    # joint_dic[stack[-1].split()[1]].add_children(joint_dic[stack[-1].split()[1] + '_end'])
                    joint_dic[stack[-1].split()[1] + '_end'].add_parent(joint_dic[stack[-1].split()[1]])
                else: 
                    for i in info:
                        if 'JOINT' in i:
                            # joint_dic[stack[-1].split()[1]].add_children(joint_dic[info[0].split()[1]])
                            joint_dic[i.split()[1]].add_parent(joint_dic[stack[-1].split()[1]])

    # print(joint_dic.keys())
    # print(joint_name)
    for i in joint_name:
        if 'Root' in i:
            joint_parent.append(-1)
            assert 'OFFSET' == joint_dic[i].get_info()[0][-1].split()[0]
            offset = np.array(joint_dic[i].get_info()[0][-1].split()[1:], dtype=np.float64)
            joint_offset.append(offset)
            continue
        for k in joint_dic.keys():
            if i == k:
                joint_parent.append(joint_name.index(joint_dic[k].get_parent()[0].get_name()))
                assert 'OFFSET' == joint_dic[i].get_info()[0][-1].split()[0]
                offset = np.array(joint_dic[i].get_info()[0][-1].split()[1:], dtype=np.float64)
                joint_offset.append(offset)
        
    return joint_name, joint_parent, joint_offset


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    joint_positions = []
    joint_orientations = []
    motion_frame_data = motion_data[frame_id]
    end_count = 0
    for i in range(len(joint_name)):
        if 'Root' in joint_name[i]:
            joint_positions.append(joint_offset[i] + motion_frame_data[:3])
            rotation = R.from_euler('XYZ', motion_frame_data[3:6], degrees=True).as_quat()
            joint_orientations.append(rotation)
        elif 'end' not in joint_name[i]:
            rotation_parent = R.from_quat(joint_orientations[joint_parent[i]]).as_matrix()
            translation = joint_positions[joint_parent[i]] + rotation_parent @ joint_offset[i].T
            rotation = R.from_euler('XYZ', motion_frame_data[3+3*(i-end_count):6+3*(i-end_count)], degrees=True).as_matrix()
            joint_positions.append(translation)
            joint_orientations.append(R.from_matrix(rotation_parent @ rotation).as_quat())
        else:
            rotation_parent = R.from_quat(joint_orientations[joint_parent[i]]).as_matrix()
            translation = joint_positions[joint_parent[i]] + rotation_parent @ joint_offset[i].T
            joint_positions.append(translation)
            joint_orientations.append(joint_orientations[joint_parent[i]])
            end_count += 1
            
    
    joint_orientations = np.array(joint_orientations)
    joint_positions = np.array(joint_positions)
    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """
    motion_data = []
    joint_data_A = {}
    joint_name_T, joint_parent_T, joint_offset_T = part1_calculate_T_pose(T_pose_bvh_path)
    joint_name_A, joint_parent_A, joint_offset_A = part1_calculate_T_pose(A_pose_bvh_path)
    motion_data_A = load_motion_data(A_pose_bvh_path)
    motion_data_T = np.zeros_like(motion_data_A)
    
    end_count_A = 0
    for i in range(len(joint_name_A)):
        if 'Root' in joint_name_A[i]:
            joint_data_A[joint_name_A[i]] = motion_data_A[:, :6]
        elif 'end' not in joint_name_A[i]:
            joint_data_A[joint_name_A[i]] = motion_data_A[:, 3+3*(i-end_count_A):6+3*(i-end_count_A)]
        else:
            end_count_A += 1
    
    rotation_l = R.from_euler('XYZ', [0, 0, 45], degrees=True)
    rotation_lp = R.from_euler('XYZ', [0, 0, 0], degrees=True)
    parent_ls_index = joint_name_A.index('lShoulder')
    while(parent_ls_index in joint_parent_A):
        new_parent_angle = (rotation_lp * R.from_euler('XYZ', joint_data_A[joint_name_A[parent_ls_index]], degrees=True) * rotation_l.inv()).as_euler('XYZ', degrees=True)
        joint_data_A[joint_name_A[parent_ls_index]] = new_parent_angle
        parent_ls_index = joint_parent_A.index(parent_ls_index)
        rotation_lp = rotation_l
    
    rotation_r = R.from_euler('XYZ', [0, 0, -45], degrees=True)
    rotation_rp = R.from_euler('XYZ', [0, 0, 0], degrees=True)
    parent_rs_index = joint_name_A.index('rShoulder')
    while(parent_rs_index in joint_parent_A):
        new_parent_angle = (rotation_rp * R.from_euler('XYZ', joint_data_A[joint_name_A[parent_rs_index]], degrees=True) * rotation_r.inv()).as_euler('XYZ', degrees=True)
        joint_data_A[joint_name_A[parent_rs_index]] = new_parent_angle
        parent_rs_index = joint_parent_A.index(parent_rs_index)
        rotation_rp = rotation_r
    
    end_count_T = 0
    for i in range(len(joint_name_T)):
        if 'Root' in joint_name_T[i]:
            motion_data_T[:, :6] = joint_data_A[joint_name_T[i]]
        elif 'end' not in joint_name_T[i]:
            motion_data_T[:, 3+3*(i-end_count_T):6+3*(i-end_count_T)] = joint_data_A[joint_name_T[i]]
        else:
            end_count_T += 1
        

    motion_data = motion_data_T
    return motion_data
