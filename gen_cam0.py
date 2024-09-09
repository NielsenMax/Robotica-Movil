import csv
import numpy as np
import pandas as pd
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

matrix = np.array([[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
                   [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
                   [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
                   [0.0, 0.0, 0.0, 1.0]])
cam_to_body = np.linalg.inv(matrix)

def b0_to_world(first_row):
    t = first_row[1:4]
    q = first_row[4:8]

    return matrix_from_pose(t,q)

def algorithm(inpath,outpath):

    ground_df = pd.read_csv(inpath)
    new_data = []

    w_t_b0 = b0_to_world(ground_df.iloc[0])
    b0_t_w = np.linalg.inv(w_t_b0)
    c0_t_w = np.dot(cam_to_body, b0_t_w)

    for row in ground_df.iterrows():
        translation_imu = np.array(row[1][1:4])
        quaternion_imu = np.array(row[1][4:8])
        timestamp = row[1][0] / 1e9

        bi_to_w = matrix_from_pose(translation_imu,quaternion_imu)

        C0_e_Ci = np.dot(np.dot(c0_t_w, bi_to_w), matrix)

        cam0_t, cam0_q = matrix_to_pose(C0_e_Ci)

        new_data.append([
            float(timestamp),
            float(cam0_t[0]),
            float(cam0_t[1]),
            float(cam0_t[2]),
            float(cam0_q[0]),
            float(cam0_q[1]),
            float(cam0_q[2]),
            float(cam0_q[3])])
    with open(outpath, mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=["timestamp", "x", "y", "z", "qw", "qx", "qy", "qz"])
        
        writer.writeheader()
        for row in new_data:
            writer.writerow({
                "timestamp":row[0],
                "x":  row[1],
                "y":  row[2],
                "z":  row[3],
                "qw": row[4],
                "qx": row[5],
                "qy": row[6],
                "qz": row[7]})
        

if __name__ == "__main__":
    algorithm("mav0/state_groundtruth_estimate0/data.csv","mav0/cam0/pose_data.csv")
