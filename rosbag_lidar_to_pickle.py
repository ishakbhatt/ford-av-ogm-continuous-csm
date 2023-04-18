# Script to convert lidar scans from Rosbag to pickled poses
#
# Author: Isha Bhatt, Meghana Kowsika, Yipeng Lin, Yanjun Chen, Thirumalaesh Ashokkumar
# Date: 04/19/2023
import sys
import velodyne_decoder as vd
import numpy as np
import os
import pickle
sys.path.append('.')
import yaml

with open("config/settings.yaml", 'r') as stream:
    param = yaml.safe_load(stream)

config = vd.Config(model='HDL-32E', gps_time=True)
rosbag_directory = param['rosbag_directory']
lidar_topics = ['/lidar_red_scan']

cloud_arrays = {}
for stamp, points, topic in vd.read_bag(rosbag_directory, config, lidar_topics):

    indices = np.where(np.logical_and(points[:,2] >= -1, points[:,2] <= 0))
    points = points[indices]

    n_points = 2000  # Number of points to plot
    idx = np.random.choice(len(points), size=n_points, replace=False)
    points = points[idx]

    # Check if the 'stamp' key exists in cloud_arrays
    if stamp in cloud_arrays:
        # Append the new points to the existing list
        cloud_arrays[stamp].extend(points)
    else:
        # Create a new key-value pair in the dictionary
        cloud_arrays[stamp] = [points]

data_directory = param['data_directory']
with open(os.path.join(data_directory, "laserScan.pkl"), "wb") as f:
        pickle.dump(cloud_arrays, f)

