import yaml


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