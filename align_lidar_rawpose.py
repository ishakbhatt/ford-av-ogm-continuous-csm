# Script to combine raw poses and laser scans into one pkl file
#
# Author: Isha Bhatt, Meghana Kowsika, Yipeng Lin, Yanjun Chen, Thirumalaesh Ashokkumar
# Date: 04/19/2023

import os
import sys
sys.path.append('.')
import yaml
import numpy as np
import pickle

with open("config/settings.yaml", 'r') as stream:
    param = yaml.safe_load(stream)

def find_closest_timestamp(target_time, timestamps):
    # Method for finding the closest time stamp
    closest_time = None
    min_diff = 100.

    for stamp in timestamps:
        diff = abs(stamp.to_sec() - target_time.to_sec())
        if diff < min_diff:
            min_diff = diff
            closest_time = stamp

    return closest_time

def main():
    # variables for unpacking pickled scans and poses
    cloud_arrays = None
    poses = None

    # unpack data from pickle files into cloud_arrays and poses
    data_directory = param['data_directory']
    with open(os.path.join(data_directory, "laserScan.pkl"), "rb") as f:
        cloud_arrays = pickle.load(f)

    with open(os.path.join(data_directory, "poses.pkl"), "rb") as f:
        poses = pickle.load(f)

    laserScan = []
    filtered_pose = []

    for i, (stamp, points) in enumerate(cloud_arrays.items()):
        points = np.array(points[0])

        # find closest time stamp for laser scans
        closest_timestamp = find_closest_timestamp(stamp, list(poses.keys()))

        # add laser scan and pose to list to pickle later
        laserScan.append([np.array([points[:, 0], points[:, 1]])])
        filtered_pose.append(poses[closest_timestamp][0])

    # convert to array
    filtered_pose = np.array(filtered_pose)

    # format to 
    robotPose = {"x": [[filtered_pose[:, 0].reshape((-1,1))]], "y": [[filtered_pose[:, 1].reshape((-1,1))]], "h": [[filtered_pose[:, 2].reshape((-1,1))]]}

    # put both 
    data_aligned = {"robotPose": robotPose, "laserScan": laserScan}
    with open(os.path.join(data_directory, "data_aligned.pkl"), "wb") as f:
        pickle.dump(data_aligned, f)

if __name__ == '__main__':
    main()