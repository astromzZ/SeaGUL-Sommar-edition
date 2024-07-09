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

# Create Figure and 3D Axes
fig = plt.figure(figsize=(10, 7))  # Adjust the figure size if needed
ax = fig.add_subplot(111, projection='3d')

# Plot the 3D data
ax.scatter(X, Y, Z, c='black', marker='o', label='3D Data')

# Plot the XY plane projection
ax.scatter(X, Y, -500, c='red', marker='o', alpha=0.6, label='XY Plane Projection')

# Plot the XZ plane projection
ax.scatter(X, [300]*len(Y), Z, c='blue', marker='o', alpha=0.6, label='XZ Plane Projection')

# Plot the YZ plane projection
ax.scatter([300]*len(X), Y, Z, c='green', marker='o', alpha=0.6, label='YZ Plane Projection')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Scatter Plot with Projections')

# Add legend
ax.legend()

# Show grid
ax.grid(True)

plt.show()
