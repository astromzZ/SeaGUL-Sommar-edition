import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.optimize import minimize
from scipy import stats
import os

file_path = "Testkod\Enskilda_delsystem\Orienteringssensor\Kalibreringstest\Example9_DMP_MultipleSensors\data.txt"

if not os.path.isfile(file_path):
    print(f"Error: File '{file_path}' not found. Please check the path.")
    exit(1)
try:
    data = pd.read_csv(file_path)
    if not set(["X", "Y", "Z"]).issubset(data.columns):
        data.columns = ["X", "Y", "Z"]
except pd.errors.EmptyDataError:
    print(f"Error: File '{file_path}' is empty.")
    exit(1)
except pd.errors.ParserError as e:
    print(f"Error parsing '{file_path}': {e}")
    exit(1)

# Extract Coordinates
X = data["X"].values
Y = data["Y"].values
Z = data["Z"].values
data_combined = np.vstack((X, Y, Z)).T

# Filtering using Z-scores
def remove_outliers(data, threshold):
    z_scores = np.abs(stats.zscore(data))
    return (z_scores < threshold).all(axis=1)

# the ellipsoid fitting function
def ellipsoid_error(params, data):
    center = params[:3]
    radii = params[3:6]
    rotation = params[6:].reshape(3, 3)
    transformed_data = np.dot(data - center, rotation) / radii
    return np.sum((np.linalg.norm(transformed_data, axis=1) - 1) ** 2)

# Iterative Filtering, Centering, and Fitting
max_iterations = 3
threshold = 2.5  # Z-score threshold for filtering
filtered_data = data_combined

for iteration in range(max_iterations):
   
    mask = remove_outliers(filtered_data, threshold)
    filtered_data = filtered_data[mask]

    center_of_mass = np.mean(filtered_data, axis=0)
    data_centered = filtered_data - center_of_mass

    initial_guess = np.concatenate([np.zeros(3), np.ones(3), np.eye(3).ravel()])
    result = minimize(ellipsoid_error, initial_guess, args=(data_centered,), method='Nelder-Mead')
    center, radii, rotation = result.x[:3], result.x[3:6], result.x[6:].reshape(3, 3)

    # Transform the data
    data_corrected = np.dot((data_centered - center), rotation) / radii
    filtered_data = data_corrected + center_of_mass  # Update filtered_data for next iteration

    # Visualize the intermediate result
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(filtered_data[:, 0], filtered_data[:, 1], filtered_data[:, 2], c='black', marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Iteration {iteration + 1} Result')
    plt.show()

# Final data after ellipsoid correction
final_center_of_mass = np.mean(filtered_data, axis=0)
data_final = filtered_data - final_center_of_mass

# Separate the final corrected data
X_final = data_final[:, 0]
Y_final = data_final[:, 1]
Z_final = data_final[:, 2]

# Create the corrected data with offset subtracted and divided by scale
corrected_data = (data_combined - center) / radii

# Plotting the calibrated data with projections
fig = plt.figure(figsize=(21, 7))
ax1 = fig.add_subplot(131, projection='3d')
ax2 = fig.add_subplot(132, projection='3d')
ax3 = fig.add_subplot(133, projection='3d')

# Plot the calibrated 3D data
ax1.scatter(X_final, Y_final, Z_final, c='black', marker='o', label='Calibrated 3D Data')
ax1.scatter(X_final, Y_final, -np.max(Z_final) - 1, c='red', marker='o', alpha=0.6, label='XY Plane Projection')
ax1.scatter(X_final, np.max(Y_final) + 1, Z_final, c='blue', marker='o', alpha=0.6, label='XZ Plane Projection')
ax1.scatter(np.max(X_final) + 1, Y_final, Z_final, c='green', marker='o', alpha=0.6, label='YZ Plane Projection')

# Set labels and title
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.set_title('Calibrated 3D Scatter Plot with Projections')
ax1.legend()
ax1.grid(True)

# Plot the original 3D data
ax2.scatter(X, Y, Z, c='black', marker='o', label='Original 3D Data')
ax2.scatter(X, Y, -np.max(Z) - 10, c='red', marker='o', alpha=0.6, label='XY Plane Projection')
ax2.scatter(X, np.max(Y) + 10, Z, c='blue', marker='o', alpha=0.6, label='XZ Plane Projection')
ax2.scatter(np.max(X) + 10, Y, Z, c='green', marker='o', alpha=0.6, label='YZ Plane Projection')

# Set labels and title
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
ax2.set_title('Original 3D Scatter Plot with Projections')
ax2.legend()
ax2.grid(True)

# Plot the corrected 3D data
ax3.scatter(corrected_data[:, 0], corrected_data[:, 1], corrected_data[:, 2], c='black', marker='o', label='Corrected 3D Data')
ax3.scatter(corrected_data[:, 0], corrected_data[:, 1], -np.max(corrected_data[:, 2]) - 1, c='red', marker='o', alpha=0.6, label='XY Plane Projection')
ax3.scatter(corrected_data[:, 0], np.max(corrected_data[:, 1]) + 1, corrected_data[:, 2], c='blue', marker='o', alpha=0.6, label='XZ Plane Projection')
ax3.scatter(np.max(corrected_data[:, 0]) + 1, corrected_data[:, 1], corrected_data[:, 2], c='green', marker='o', alpha=0.6, label='YZ Plane Projection')

# Set labels and title
ax3.set_xlabel('X')
ax3.set_ylabel('Y')
ax3.set_zlabel('Z')
ax3.set_title('Corrected 3D Scatter Plot with Projections')
ax3.legend()
ax3.grid(True)

plt.show()

# 1. Extract Hard Iron Offset
hard_iron_offset = center

# 2. Extract Soft Iron Correction
soft_iron_scale = radii
soft_iron_rotation = rotation

print(f"Hard Iron Offset: {hard_iron_offset}")
print(f"Soft Iron Scale (Radii): {soft_iron_scale}")
print(f"Soft Iron Rotation Matrix:\n{soft_iron_rotation}")
