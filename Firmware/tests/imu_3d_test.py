import matplotlib.pyplot as plt
import numpy as np

# Load CSV, skip the header rows
data = np.loadtxt("transform.csv", delimiter=",")

# Extract translation vector (last column)
t = data[:, 3]

# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Origin
origin = np.array([0, 0, 0])
ax.scatter(*origin, color='blue', label='Origin')

# Draw translation vector
ax.quiver(
    origin[0], origin[1], origin[2],  # start point
    t[0], t[1], t[2],                # vector components
    color='red',
    arrow_length_ratio=0.1,
    linewidth=2,
    label='Translation'
)

# Set labels and title
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Translation Vector")
ax.legend()
ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio

# Save figure and show
plt.savefig("translation_vector.png")
plt.show()
