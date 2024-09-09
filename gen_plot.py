import numpy as np
import pandas as pd
import transforms3d.affines as ta
import transforms3d.quaternions as tq
import matplotlib.pyplot as plt
from common import b0_to_world, matrix_from_pose, matrix_to_pose, cam0_path, imu_ground_truth_path

def plot_pose(ax, translation, quaternion, color):

    ax.scatter(translation[0], translation[1], translation[2], color=color)

    forward_direction = tq.rotate_vector(np.array([1,0,0]), quaternion)

    ax.quiver(translation[0], translation[1], translation[2], 
          forward_direction[0], forward_direction[1], forward_direction[2], 
          length=0.01, color='g')
    
def plot_poses(imu_file_path, cam_file_path, ax, num_poses, step):
    
    imu_file = pd.read_csv(imu_file_path)
    cam_file = pd.read_csv(cam_file_path)

    imu_rows = imu_file.iloc
    cam_rows = cam_file.iloc

    num_poses = min(num_poses, len(imu_file), len(cam_file))

    w_t_b0 = b0_to_world(imu_rows[0])


    for i in range(0, num_poses, step):
        imu_t = imu_rows[i][1:4]
        imu_q = imu_rows[i][4:8]

        cam0_t_wrt_c0 = cam_rows[i][1:4]
        cam0_q_wrt_c0 = cam_rows[i][4:8]

        C0_T_Ci = matrix_from_pose(cam0_t_wrt_c0,cam0_q_wrt_c0)

        W_T_Ci = np.dot(w_t_b0,C0_T_Ci)

        cam0_t, cam0_q = matrix_to_pose(W_T_Ci)

        plot_pose(ax, imu_t.iloc, imu_q, color='b')
        plot_pose(ax, cam0_t, cam0_q, color='r')

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    plot_poses(imu_ground_truth_path,cam0_path,ax, 1000, 10)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    robot_handle = ax.scatter([], [], [], color='r', label='Robot')
    camera_handle = ax.scatter([], [], [], color='b', label='Camera')

    ax.legend(handles=[robot_handle, camera_handle])

    plt.show()


if __name__ == "__main__":
    main()