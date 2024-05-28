import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_3d_coordinates(x, y, z):
    # Ensure there are exactly 9 values in each array
    if len(x) != 9 or len(y) != 9 or len(z) != 9:
        raise ValueError("Each of the three arrays must contain exactly 9 values")

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the points
    ax.scatter(x, y, z, c='b', marker='o')

    # Optional: Add labels to the points
    for i in range(len(x)):
        ax.text(x[i], y[i], z[i], f'({x[i]:.2f}, {y[i]:.2f}, {z[i]:.2f})', fontsize=9)

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

plot_3d_coordinates(x, y, z)
