import velodyne_decoder as vd
import numpy as np
import os
import pickle
import matplotlib.pyplot as plt


config = vd.Config(model='VLP-32C', gps_time=True)
bagfile = '/home/coned/data/Sample-Data.bag'
lidar_topics = ['/lidar_blue_scan', '/lidar_green_scan', '/lidar_red_scan', '/lidar_yellow_scan']

cloud_arrays = {}
for stamp, points, topic in vd.read_bag(bagfile, config, lidar_topics):
    # print(points.shape)
    indices = np.where(np.logical_and(points[:,3] >= 70. ,np.logical_and(points[:,2] >= -2, points[:,2] <= 2)))
    points = points[indices]

    # data = points[:, 3]
    # mean = np.mean(data)
    # variance = np.var(data)
    # # Compute the histogram of the data using NumPy
    # hist, bin_edges = np.histogram(data, bins=10)

    # # Create a histogram plot of the data using Matplotlib
    # plt.hist(data, bins=10)
    # plt.title("Histogram of Data")
    # plt.xlabel("Value")
    # plt.ylabel("Frequency")

    # # Add frequency counts as text annotations on the plot
    # for i in range(len(hist)):
    #     plt.text(bin_edges[i], hist[i], str(hist[i]), ha='center', va='bottom')

    # plt.show()

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    intensity = points[:, 4]

    n_points = 1000  # Number of points to plot
    idx = np.random.choice(len(x), size=n_points, replace=False)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    scatter = ax.scatter(x[idx], y[idx], z[idx], c=intensity[idx], cmap='viridis')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    cbar = plt.colorbar(scatter)
    cbar.set_label('Ring')

    # ax.set_xlim([-1, 1])
    # ax.set_ylim([-1, 1])

    # # show the plot
    plt.show()

    # Check if the 'stamp' key exists in cloud_arrays
    if stamp in cloud_arrays:
        # Append the new points to the existing list
        cloud_arrays[stamp].extend(points)
    else:
        # Create a new key-value pair in the dictionary
        cloud_arrays[stamp] = [points]



data_directory = "./data/"
with open(os.path.join(data_directory, "laserScan.pkl"), "wb") as f:
        pickle.dump(cloud_arrays, f)

