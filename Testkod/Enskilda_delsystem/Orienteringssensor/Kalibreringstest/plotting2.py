import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1. File Path
file_path = "your_data_file.txt"  # Replace with the correct path

# 2. Read Data with Pandas (Assuming space-separated values and no header)
data = pd.read_csv(file_path, sep=",", header=None, names=["X", "Y", "Z"])

# 3. Extract Columns (No need to split manually)
X = data["X"]
Y = data["Y"]
Z = data["Z"]

# Create 3D plot (Same as before)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(X, Y, Z, c='r', marker='o')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()