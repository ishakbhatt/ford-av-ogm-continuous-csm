# Script to convert raw pose from Rosbag to pickled poses
#
# Author: Isha Bhatt, Meghana Kowsika, Yipeng Lin, Yanjun Chen, Thirumalaesh Ashokkumar
# Date: 04/19/2023

import os
import rospy
import pickle
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
import rosbag
import subprocess

sys.path.append('.')
import yaml
with open("config/settings.yaml", 'r') as stream:
    param = yaml.safe_load(stream)

# global variables to be used across functions and calls of callback
laser_scan_subscriber = None
timer = None

data_directory = param['data_directory']
rosbag_directory = param['rosbag_directory']
rosbag_play = "rosbag play " + rosbag_directory

poses = {}

def filtered_pose_callback(msg):
    print("Reading pose")
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
    rospy.Subscriber('/pose_ground_truth', PoseStamped, filtered_pose_callback)

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