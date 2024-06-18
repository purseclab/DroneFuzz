import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def cartesian_to_spherical(x, y, z):
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(z / r)  # polar angle
    phi = np.arctan2(y, x)    # azimuthal angle
    return r, theta, phi

def plot_spherical_coordinates(x, y, z):
    # Ensure there are exactly 9 values in each array
    if len(x) != 9 or len(y) != 9 or len(z) != 9:
        raise ValueError("Each of the three arrays must contain exactly 9 values")
    
    # Convert Cartesian coordinates to spherical coordinates
    r, theta, phi = cartesian_to_spherical(np.array(x), np.array(y), np.array(z))
    
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the points in spherical coordinates
    ax.scatter(r * np.sin(theta) * np.cos(phi), 
               r * np.sin(theta) * np.sin(phi), 
               r * np.cos(theta), 
               c='b', marker='o')
    
    # Optional: Add labels to the points
    for i in range(len(x)):
        ax.text(r[i] * np.sin(theta[i]) * np.cos(phi[i]), 
                r[i] * np.sin(theta[i]) * np.sin(phi[i]), 
                r[i] * np.cos(theta[i]), 
                f'({x[i]:.2f}, {y[i]:.2f}, {z[i]:.2f})', fontsize=9)
    
    # Set labels for the axes
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    
    # Display the plot
    plt.show()

# Example data
x = [0.6992953230783125, 0.7987399163263875, 2.077960547631796, -1.0718840937011624, 1.0662852720944445, 0.2905720025592357, -0.6496419248711868, 2.1102699277107964, 2.165948458554089]
y = [0.6992953230783125, 0.7987399163263875, 2.077960547631796, -1.0718840937011624, 1.0662852720944445, 0.2905720025592357, -0.6496419248711868, 2.1102699277107964, 2.165948458554089]
z = [0.6992953230783125, 0.7987399163263875, 2.077960547631796, -1.0718840937011624, 1.0662852720944445, 0.2905720025592357, -0.6496419248711868, 2.1102699277107964, 2.165948458554089]

plot_spherical_coordinates(x, y, z)
