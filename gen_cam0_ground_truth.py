import numpy as np
import pandas as pd
import transforms3d.affines as ta
import transforms3d.quaternions as tq
from poses import * 

def matrix_from_pose(t,q):
    transformation_matrix = ta.compose(t, tq.quat2mat(q), np.ones(3))

    return transformation_matrix

def matrix_to_pose(matrix):
    translation, rotation_matrix, _, _ = ta.decompose44(matrix)
    quaternion = tq.mat2quat(rotation_matrix)

    translation = [round(t, 6) for t in translation]
    quaternion = [round(q, 6) for q in quaternion]
    return translation,quaternion

def algorithm(inpath,outpath):

    ground_df = pd.read_csv(inpath)
    new_data = []
    for row in ground_df.iterrows():
        translation_imu = np.array(row[1][1:4])
        quaternion_imu = np.array(row[1][4:8])
        timestamp = row[1] / 1e9

        W_T_Bi = matrix_from_pose(translation_imu,quaternion_imu)

        C0_e_Ci = np.dot(np.dot(C0_T_W, W_T_Bi), B_T_C)

        cam0_t, cam0_q = transformation_matrix_to_pose(C0_e_Ci)

        new_data.append(
            {"timestamp":timestamp,
            "x":cam0_t[0],
            "y":cam0_t[1],
            "z":cam0_t[2],
            "qw": cam0_q[0],
            "qx": cam0_q[1],
            "qy": cam0_q[2],
            "qz": cam0_q[3]})
    
    
    

if __name__ == "__main__":
    algorithm("mav0/state_groundtruth_estimate0/data.csv","mav0/cam0/pose_data.csv")
