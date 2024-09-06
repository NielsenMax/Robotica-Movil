import pandas as pd
import numpy as np
import transforms3d.quaternions as tq

file_path = '/run/media/lau/Datos/Datos/LCC/Robotica/TP1/mav0/state_groundtruth_estimate0/data.csv'
df = pd.read_csv(file_path)
selected_columns = df.iloc[:, :8]
selected_columns.columns = ['timestamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']

trajectory = []
for row in selected_columns.iterrows():
    timestamp = row[1][0]
    position = np.array(row[1][1:4])
    quaternion = np.array(row[1][4:8])

    rotated = tq.rotate_vector(position, quaternion)
    trajectory.append(rotated)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Example: list of 3x1 numpy arrays
#positions = [np.array([x, y, z]).reshape(3, 1) for x, y, z in zip(range(10), range(10, 20), range(20, 30))]
positions = np.array(trajectory)

# Extract X, Y, Z coordinates
x_coords = positions[:, 0]
y_coords = positions[:, 1]
z_coords = positions[:, 2]

indices = list(range(0, len(positions), round(len(positions)/1000))) + [len(positions)-1]
x_coords = x_coords[indices]
y_coords = y_coords[indices]
z_coords = z_coords[indices]

fig = plt.figure(dpi=100)
ax = fig.add_subplot(projection='3d')
ax.set(xlim=(min(x_coords), 
                max(x_coords)), 
          ylim=(min(y_coords), 
                max(y_coords)), 
          zlim=(min(z_coords), 
                max(z_coords)))
line = ax.plot(x_coords, y_coords, z_coords)[0]
def update(frame):
    print(f"Frame: {frame}")
    x = x_coords[:frame]
    y = y_coords[:frame]
    z = z_coords[:frame]

    line.set_data_3d((x, y, z))

    return line


ani = FuncAnimation(fig=fig, func=update, frames=len(indices), interval=30, repeat=True)
ani.save("trajectory.mp4", writer='ffmpeg', fps=30, extra_args=['-loop','1'])
plt.show()