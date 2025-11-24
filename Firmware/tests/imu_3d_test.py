import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # needed for 3D

data = np.loadtxt("output.csv", delimiter=",")

x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(x, y, z)

ax.set_xlabel("X position")
ax.set_ylabel("Y position")
ax.set_zlabel("Z position")
ax.set_title("2025-11-18_12-27-10_angularMovementRoomMoon90Deg")

plt.savefig("IMU_12-27-10.png")


"""import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("LOG251118_122708.txt", skiprows=1)

x = data[:, 4]
y = data[:, 5]
z = data[:, 6]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(x, y, z)

ax.set_xlabel("Robot_X [mm]")
ax.set_ylabel("Robot_Y [mm]")
ax.set_zlabel("Robot_Z [mm]")
ax.set_title("Robot 3D Trajectory LOG251118_122708")

max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0

mid_x = (x.max() + x.min()) * 0.5
mid_y = (y.max() + y.min()) * 0.5
mid_z = (z.max() + z.min()) * 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.savefig("LOG251118_122708.png")
"""
