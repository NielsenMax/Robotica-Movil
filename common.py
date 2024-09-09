import numpy as np
import transforms3d.affines as ta
import transforms3d.quaternions as tq

def matrix_from_pose(t,q):
    transformation_matrix = ta.compose(t, tq.quat2mat(q), np.ones(3))

    return transformation_matrix

def matrix_to_pose(matrix):
    translation, rotation_matrix, _, _ = ta.decompose44(matrix)
    quaternion = tq.mat2quat(rotation_matrix)

    translation = [round(t, 6) for t in translation]
    quaternion = [round(q, 6) for q in quaternion]
    return translation,quaternion

def b0_to_world(first_row):
    t = first_row[1:4]
    q = first_row[4:8]

    return matrix_from_pose(t,q)

imu_ground_truth_path = "mav0/state_groundtruth_estimate0/data.csv"
cam0_path = "mav0/cam0/pose_data.csv"