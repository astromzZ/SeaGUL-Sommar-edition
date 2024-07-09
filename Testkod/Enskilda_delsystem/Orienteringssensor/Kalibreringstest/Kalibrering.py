import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from scipy import stats

# 1. File Path
file_path = "Testkod\Enskilda_delsystem\Orienteringssensor\Kalibreringstest\Example9_DMP_MultipleSensors\data.txt"  # Replace with the actual path to your file

# 2. Read Data with Pandas
try:
    data = pd.read_csv(file_path, sep=",", header=None, names=["X", "Y", "Z"])
except pd.errors.ParserError:
    try:
        data = pd.read_csv(file_path, sep="\t", header=None, names=["X", "Y", "Z"])  # Try tab delimiter
    except pd.errors.ParserError:
        print("Error: Unable to read the data file. Please check the delimiter and file format.")
        exit(1)  # Exit gracefully if file reading fails

# 3. Extract Coordinates
X = data["X"].values
Y = data["Y"].values
Z = data["Z"].values

# Combine data
data_combined = np.vstack((X, Y, Z)).T

# 1. Hard Iron Offset Removal
offsets = (data_combined.max(axis=0) + data_combined.min(axis=0)) / 2
data_centered = data_combined - offsets

# 2. Soft Iron Correction (Ellipsoid Fitting with PCA)
pca = PCA(n_components=3)
pca.fit(data_centered)
data_corrected = pca.inverse_transform(pca.transform(data_centered))

# (Optional) 3. Outlier Removal with Z-Scores
z_scores = np.abs(stats.zscore(data_corrected))
threshold = 2  
filtered_entries = (z_scores < threshold).all(axis=1)
data_final = data_corrected[filtered_entries]

# Separate the final corrected data
X_final = data_final[:, 0]
Y_final = data_final[:, 1]
Z_final = data_final[:, 2]


# Plotting the calibrated data
fig = plt.figure(figsize=(10, 7))
ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122, projection='3d')
# Plot the 3D data
ax1.scatter(X_final, Y_final, Z_final, c='black', marker='o', label='Calibrated 3D Data')

# Plot the XY plane projection
ax1.scatter(X_final, Y_final, -500, c='red', marker='o', alpha=0.6, label='XY Plane Projection')

# Plot the XZ plane projection
ax1.scatter(X_final, [400]*len(Y_final), Z_final, c='blue', marker='o', alpha=0.6, label='XZ Plane Projection')

# Plot the YZ plane projection
ax1.scatter([400]*len(X_final), Y_final, Z_final, c='green', marker='o', alpha=0.6, label='YZ Plane Projection')

# Set labels and title
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.set_title('Calibrated 3D Scatter Plot with Projections')

# Add legend
ax1.legend()
# Show grid
ax1.grid(True)

# Plot the 3D data
ax2.scatter(X, Y, Z, c='black', marker='o', label='3D Data')

# Plot the XY plane projection
ax2.scatter(X, Y, -500, c='red', marker='o', alpha=0.6, label='XY Plane Projection')

# Plot the XZ plane projection
ax2.scatter(X, [300]*len(Y), Z, c='blue', marker='o', alpha=0.6, label='XZ Plane Projection')

# Plot the YZ plane projection
ax2.scatter([300]*len(X), Y, Z, c='green', marker='o', alpha=0.6, label='YZ Plane Projection')

# Set labels and title
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
ax2.set_title('3D Scatter Plot with Projections')

# Add legend
ax2.legend()

# Show grid
ax2.grid(True)

plt.show()
