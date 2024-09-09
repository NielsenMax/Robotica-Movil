import pandas as pd
import numpy as np
import transforms3d.quaternions as tq
import transforms3d.affines as ta
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import transforms3d.affines as ta

matrix = np.array([[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
                   [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
                   [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
                   [0.0, 0.0, 0.0, 1.0]])

def matrix_from_pose(t,q):
    transformation_matrix = ta.compose(t, tq.quat2mat(q), np.ones(3))

    return transformation_matrix

def matrix_to_pose(matrix):
    translation, rotation_matrix, _, _ = ta.decompose44(matrix)
    quaternion = tq.mat2quat(rotation_matrix)

    translation = [round(t, 6) for t in translation]
    quaternion = [round(q, 6) for q in quaternion]
    return translation,quaternion

ground_path= './mav0/state_groundtruth_estimate0/data.csv'

ground_df = pd.read_csv(ground_path)

num_poses = min(len(ground_df), len(cam_df))

c_t_b = np.linalg.inv(matrix)

imu = ground_df[0]
translation_imu = np.array(imu[1:4])
quaternion_imu = np.array(imu[1][4:8])
 
w_t_b0 = matrix_from_pose(translation_imu,quaternion_imu)
b0_t_w = np.linalg.inv(w_t_b0)

w_t_c0 = np.dot(w_t_b0,matrix)
c0_t_w = np.dot(c_t_b, b0_t_w)

fig = plt.figure
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_title("IMU")
ax2 = fig.add_subplot(122, projection='3d')
ax2.set_title("Cámara")

for i in range(num_poses):
    imu = ground_df[i]  
    cam = cam_df[i]
    translation_imu = np.array(imu[1:4])
    translation_cam = np.array(cam[1:4])

    quaternion_imu = np.array(imu[1][4:8])
    quaternion_cam = np.array(cam[1][4:8])

    t = matrix_from_pose(quaternion_imu, quaternion_cam)
    w_t_ci = np.dot(w_t_c0,t)
    cam0_t, cam0_q = matrix_to_pose(w_t_ci)


    ax1.scatter(translation_imu[0], translation_imu[1], translation_imu[2])
    ax2.scatter(cam0_t[0], cam0_t[1], cam0_t[2])

plt.show()

