import csv
import numpy as np
import pandas as pd
from common import b0_to_world,matrix_from_pose,matrix_to_pose,imu_ground_truth_path,cam0_path, body_to_cam, cam_to_body


def algorithm(inpath,outpath):

    ground_df = pd.read_csv(inpath)
    new_data = []

    w_t_b0 = b0_to_world(ground_df.iloc[0])
    b0_t_w = np.linalg.inv(w_t_b0)
    w_to_c0 = np.dot(body_to_cam, b0_t_w)

    for row in ground_df.iterrows():
        translation_imu = np.array(row[1][1:4])
        quaternion_imu = np.array(row[1][4:8])
        timestamp = row[1].iloc[0] / 1e9

        bi_to_w = matrix_from_pose(translation_imu,quaternion_imu)

        C0_e_Ci = np.dot(np.dot(w_to_c0, bi_to_w), cam_to_body)

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
    algorithm(imu_ground_truth_path, cam0_path)
