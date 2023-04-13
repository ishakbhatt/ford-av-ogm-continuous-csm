import velodyne_decoder as vd
import numpy as np
import os
import pickle

config = vd.Config(model='VLP-32C', gps_time=True)
bagfile = '/home/coned/data/Sample-Data.bag'
lidar_topics = ['/lidar_blue_scan', '/lidar_green_scan', '/lidar_red_scan', '/lidar_yellow_scan']

cloud_arrays = {}
for stamp, points, topic in vd.read_bag(bagfile, config, lidar_topics):
    # print(points.shape)
    indices = np.where(np.logical_and(points[:,2] >= -2, points[:,2] <= 2))
    points = points[indices]

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

