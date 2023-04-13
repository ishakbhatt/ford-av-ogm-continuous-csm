import os
import numpy as np
import rospy
import pickle
from scipy.io import savemat
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from std_msgs.msg import Empty
import rosbag
import subprocess
from velodyne_msgs.msg import VelodyneScan, VelodynePacket
import struct

def find_closest_timestamp(target_time, timestamps):
    closest_time = None
    min_diff = 100.

    for stamp in timestamps:
        diff = abs(stamp.to_sec() - target_time.to_sec())
        if diff < min_diff:
            min_diff = diff
            closest_time = stamp

    return closest_time


cloud_arrays = None
poses = None

data_directory = "./data/"
with open(os.path.join(data_directory, "laserScan.pkl"), "rb") as f:
    cloud_arrays = pickle.load(f)

with open(os.path.join(data_directory, "poses.pkl"), "rb") as f:
    poses = pickle.load(f)

laserScan = []
flitered_pose = []

for i, (stamp, points) in enumerate(cloud_arrays.items()):
    points = np.array(points[0])

    closest_timestamp = find_closest_timestamp(stamp, list(poses.keys()))


    laserScan.append([np.array([points[:, 0], points[:, 1]])])
    flitered_pose.append(poses[closest_timestamp][0])

flitered_pose = np.array(flitered_pose)

robotPose = {"x": [[flitered_pose[:, 0].reshape((-1,1))]], "y": [[flitered_pose[:, 1].reshape((-1,1))]], "h": [[flitered_pose[:, 2].reshape((-1,1))]]}

data_aligned = {"robotPose": robotPose, "laserScan": laserScan}
with open(os.path.join(data_directory, "data_aligned.pkl"), "wb") as f:
    pickle.dump(data_aligned, f)
