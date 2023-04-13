import rospy
import pickle
import sys
import argparse
import numpy as np
from scipy.io import loadmat
import os
from utils import plot_mean, plot_variance, plot_semantic
from ogm_continuous_CSM import ogm_continuous_CSM
from scipy.spatial.transform import Rotation as R


# subscribe to topics for pose and scan
# save info to dictionary with robotPose and laserScan
'''def laser_scan_callback(msg):
    header = msg.header
    packets = msg.packets

def filtered_pose_callback(msg): # 
    # x and y
    x = msg.pose.position.x
    y = msg.pose.position.y

    # quaternion angles to euler
    x_quat = msg.pose.orientation.x
    y_quat = msg.pose.orientation.y
    z_quat = msg.pose.orientation.z
    w_quat = msg.pose.orientation.w

    # rotation
    r = R.from_quat([x_quat, y_quat, z_quat, w_quat])
    eulXYZ = r.as_euler('xyz')

    # extract yaw
    yaw = eulXYZ[2]'''


def main():
    # subscribe for pose and scans
    #filtered_pose_subscriber = rospy.Subscriber('/pose_ground_truth', PoseStamped, filtered_pose_callback)
    #laser_scan_subscriber = rospy.Subscriber('/lidar_red_scan', VelodyneScan, laser_scan_callback) 

    data_directory = "/home/coned/data/"
    with open(os.path.join(data_directory, "data_aligned.pkl"), "rb") as f:
        data_aligned = pickle.load(f)

    robotPose = data_aligned['robotPose']
    laserScan = data_aligned['laserScan']

    # continuous S-CSM
    ogm = ogm_continuous_CSM()````
        
    # build map
    ogm.construct_map(robotPose, laserScan)
    ogm.build_ogm()

    # plot
    plot_semantic(ogm, 'Continuous S-CSM Mean', 'ogm_intel_continuous_S_CSM_mean.png')
    plot_variance(ogm, 'Continuous S-CSM Variance (grid size = 0.135)', 'ogm_intel_continuous_S_CSM_variance_grid_0135.png')


if __name__ == '__main__':
    main()
