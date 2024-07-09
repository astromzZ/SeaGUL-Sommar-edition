import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1. File Path 
file_path = "Testkod\Enskilda_delsystem\Orienteringssensor\Kalibreringstest\Example9_DMP_MultipleSensors\data.txt"  # Replace with the actual path to your file

# 2. Read Data with Pandas
# Handle potential delimiter issues (e.g., tabs, commas)
try:
    data = pd.read_csv(file_path, sep=",", header=None, names=["X", "Y", "Z"])
except pd.errors.ParserError:
    try:
        data = pd.read_csv(file_path, sep="\t", header=None, names=["X", "Y", "Z"])  # Try tab delimiter
    except pd.errors.ParserError:
        print("Error: Unable to read the data file. Please check the delimiter and file format.")
        exit(1)  # Exit gracefully if file reading fails

# 3. Extract Coordinates 
X = data["X"]
Y = data["Y"]
Z = data["Z"]

# Create Figure and Subplots 
fig = plt.figure(figsize=(15, 5))  # Wider figure for clearer projections

# 3D Scatter Plot (Main Plot)
ax = fig.add_subplot(131, projection='3d')  
ax.scatter(X, Y, Z, c='purple', marker='o', label='3D Data')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
ax.set_title('3D Scatter Plot')
ax.legend()

# XY Plane Projection
ax1 = fig.add_subplot(132, projection='3d')
ax1.scatter(X, Y, 0, c='r', marker='o')
ax1.scatter(X, Y, Z, c='purple', marker='o', alpha=0.2)  # Show 3D points faintly
ax1.set_xlabel('X Label')
ax1.set_ylabel('Y Label')
ax1.set_zlabel('Z Label')
ax1.set_title('XY Plane Projection')

# XZ Plane Projection
ax2 = fig.add_subplot(133, projection='3d')
ax2.scatter(X, 0, Z, c='b', marker='o')
ax2.scatter(X, Y, Z, c='purple', marker='o', alpha=0.2)  # Show 3D points faintly
ax2.set_xlabel('X Label')
ax2.set_ylabel('Y Label')
ax2.set_zlabel('Z Label')
ax2.set_title('XZ Plane Projection')


plt.tight_layout()
plt.show()
