import re
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np

# Function to parse a line of the log file
def parse_line(line):
    # Regular expression to extract data from the log line
    pattern = re.compile(r'builtin_interfaces\.msg\.Time\(sec=(\d+), nanosec=(\d+)\)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)')
    match = pattern.match(line)
    if match:
        # Extract data from the match groups
        sec = int(match.group(1))
        nanosec = int(match.group(2))
        x = float(match.group(3))
        y = float(match.group(4))
        orientation = float(match.group(5))
        linear_velocity = float(match.group(6))
        angular_velocity = float(match.group(7))
        
        if linear_velocity == 0.0 and angular_velocity == 0.0:
            return None

        # Create a timestamp from sec and nanosec
        timestamp = datetime.fromtimestamp(sec + nanosec / 1e9)
        
        return {
            'timestamp': timestamp,
            'x': x,
            'y': y,
            'orientation': orientation,
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity
        }
    else:
        return None

# Function to parse the log file
def parse_log_file(file_path):
    parsed_data = []
    with open(file_path, 'r') as file:
        for line in file:
            parsed_line = parse_line(line)
            if parsed_line:
                parsed_data.append(parsed_line)
    return parsed_data

def plot_xy_path(parsed_data):
    x_coords = [data['x'] for data in parsed_data]
    y_coords = [data['y'] for data in parsed_data]
    
    plt.figure(figsize=(10, 6))
    plt.plot(x_coords, y_coords, marker='o')
    plt.title('XY Path')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)
    plt.show()

# Function to plot the xy path with orientation arrows
def plot_pose(parsed_data, step = 20):
    x_coords = [data['x'] for data in parsed_data]
    y_coords = [data['y'] for data in parsed_data]
    orientations = [data['orientation'] for data in parsed_data]
    
    plt.figure(figsize=(10, 6))
    # plt.scatter(x_coords, y_coords, c='blue', label='Position')
    plt.scatter(x_coords[0], y_coords[0], c='blue', label='Start')
    plt.scatter(x_coords[-1], y_coords[-1], c='green', label='End')
    
    for i in range(0, len(x_coords), step):
        x = x_coords[i]
        y = y_coords[i]
        orientation = orientations[i]
        dx = np.cos(orientation)
        dy = np.sin(orientation)

        plt.arrow(x, y, dx * 0.1, dy * 0.1, head_width=0.05, head_length=0.1, fc='red', ec='red')
    
    plt.title('Pose Path')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)
    plt.legend()
    plt.show()


# Function to plot linear velocities over time
def plot_linear_velocity(parsed_data):
    timestamps = [data['timestamp'] for data in parsed_data]
    linear_velocities = [data['linear_velocity'] for data in parsed_data]
    
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, linear_velocities, color='tab:blue', label='Linear Velocity')
    plt.xlabel('Time')
    plt.ylabel('Linear Velocity')
    plt.title('Linear Velocity Over Time')
    plt.grid(True)
    plt.legend()
    plt.show()

# Function to plot angular velocities over time
def plot_angular_velocity(parsed_data):
    timestamps = [data['timestamp'] for data in parsed_data]
    angular_velocities = [data['angular_velocity'] for data in parsed_data]
    
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, angular_velocities, color='tab:red', label='Angular Velocity')
    plt.xlabel('Time')
    plt.ylabel('Angular Velocity')
    plt.title('Angular Velocity Over Time')
    plt.grid(True)
    plt.legend()
    plt.show()

log_file_path = 'log.txt'
parsed_data = parse_log_file(log_file_path)
# plot_xy_path(parsed_data)
# plot_pose(parsed_data)
plot_linear_velocity(parsed_data)
plot_angular_velocity(parsed_data)