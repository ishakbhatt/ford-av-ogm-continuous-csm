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


# global variables to append to for every call of callbacks
laser_scan_subscriber = None
timer = None

data_directory = "/home/coned/data/"
rosbag_directory = "/home/coned/data/Sample-Data.bag"
rosbag_play = "rosbag play " + rosbag_directory


poses = {}

def filtered_pose_callback(msg):
    # print("Reading pose")
    stamp = msg.header.stamp

    # quaternion angles to euler
    x_quat = msg.pose.orientation.x
    y_quat = msg.pose.orientation.y
    z_quat = msg.pose.orientation.z
    w_quat = msg.pose.orientation.w

    # rotation
    r = R.from_quat([x_quat, y_quat, z_quat, w_quat])
    eulXYZ = r.as_euler('xyz')


    pose = [msg.pose.position.x, msg.pose.position.y, eulXYZ[2]] #x, y, h

    if stamp in poses:
        # Append the new points to the existing list
        poses[stamp].extend(pose)
        assert(False)
    else:
        # Create a new key-value pair in the dictionary
        poses[stamp] = [pose]

 
# finished, save to .mat file
def quit_func(event):
    # shut down timer
    print("Shut down")

    # shutdown signal
    rospy.signal_shutdown("All data has been processed.")

    # pickle
    with open(os.path.join(data_directory, "poses.pkl"), "wb") as f:
        pickle.dump(poses, f)

    # finished message
    print("Finished.")

    # clean exit
    os._exit(os.EX_OK)

def main():
    # initialize listener node
    rospy.init_node('listener', anonymous=True)

    # subscribe for pose and scans
    print("Initializing pose and laser scan subscribers...")
    filtered_pose_subscriber = rospy.Subscriber('/pose_ground_truth', PoseStamped, filtered_pose_callback)

    # compute duration of playing bag
    bag = rosbag.Bag(rosbag_directory, 'r')
    duration = bag.get_end_time() - bag.get_start_time()
    bag.close()

    # calculate time remaining until rosbag finishes playing
    time_elapsed = rospy.get_time() - rospy.Time.now().to_sec()
    remaining_time = duration - time_elapsed
    timer = rospy.Timer(rospy.Duration(secs=remaining_time), quit_func)

    # do not exit until node is stopped
    subprocess.check_output(rosbag_play, shell=True)
    rospy.spin()

if __name__ == '__main__':
    main()