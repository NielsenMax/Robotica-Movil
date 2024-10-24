import yaml
import numpy as np
import transforms3d.affines as ta
import transforms3d.quaternions as tq


def read_calibation_data(path):
        # Load the YAML file
        with open(path, 'r') as file:
            camera_info = yaml.safe_load(file)

        # Extract the information
        image_width = camera_info['image_width']
        image_height = camera_info['image_height']
        camera_name = camera_info['camera_name']

        camera_matrix = camera_info['camera_matrix']['data']
        distortion_model = camera_info['distortion_model']
        distortion_coefficients = camera_info['distortion_coefficients']['data']
        rectification_matrix = camera_info['rectification_matrix']['data']
        projection_matrix = camera_info['projection_matrix']['data']

        return {
            'image_width': image_width,
            'image_height': image_height,
            'camera_name': camera_name,
            'camera_matrix': camera_matrix,
            'distortion_model': distortion_model,
            'distortion_coefficients': distortion_coefficients,
            'rectification_matrix': rectification_matrix,
            'projection_matrix': projection_matrix,
        }
        
def matrix_from_pose(t,q):
    transformation_matrix = ta.compose(t, tq.quat2mat(q), np.ones(3))

    return transformation_matrix
        
left_cam_to_body = np.array([[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
                            [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
                            [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
                            [0.0, 0.0, 0.0, 1.0]])
body_to_right_cam = np.linalg.inv(np.array([[0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556],
                            [0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024],
                            [-0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038],
                            [0.0, 0.0, 0.0, 1.0]]))        