import os
import numpy as np
import rospy
from scipy.io import savemat
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from std_msgs.msg import Empty

# global variables to append to for every call of callbacks
x = []
y = []
h = []
scans = []
laser_scan_subscriber = None
timer = None

# subscribe to topics for pose and scan
# save info to dictionary with robotPose and laserScan
def laser_scan_callback(msg):
    print("Reading laser scan")
    # get x and y from point cloud message
    pt_x = []
    pt_y = []
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True): # sensor_msgs.point_cloud2.
        pt_x.append(point[0])
        pt_y.append(point[1])
    
    # reformat into packets similar to each
    # element of laserScan in original .mat file
    packets = np.vstack((pt_x, pt_y))

    # add scan to scans
    scans.append(packets)


def filtered_pose_callback(msg): 
    print("Reading pose")
    # x and y
    x.append(msg.pose.position.x)
    y.append(msg.pose.position.y)

    # quaternion angles to euler
    x_quat = msg.pose.orientation.x
    y_quat = msg.pose.orientation.y
    z_quat = msg.pose.orientation.z
    w_quat = msg.pose.orientation.w

    # rotation
    r = R.from_quat([x_quat, y_quat, z_quat, w_quat])
    eulXYZ = r.as_euler('xyz')

    # extract yaw
    h.append(eulXYZ[2])

# finished, save to .mat file
def quit_func(message):
    # shut down timer
    timer.shutdown()
    
    # unsunscribe from topics
    laser_scan_subscriber.unregister()

    # write poses and scans to mat file
    data = {"robotPose": {"x": x, "y": y, "h": h}, "laserScan": scans}
    savemat(os.path.join('data','fordAV.mat'), data)

    # finished message
    print("Finished.")

    # clean exit
    os._exit(os.EX_OK)

def main():
    # initialize listener node
    rospy.init_node('listener', anonymous=True)
    #rospy.on_shutdown(quit_func)
    # subscribe for pose and scans
    print("Initializing pose and laser scan subscribers...")
    filtered_pose_subscriber = rospy.Subscriber('/pose_ground_truth', PoseStamped, filtered_pose_callback)
    laser_scan_subscriber = rospy.Subscriber('/pointcloud_map', PointCloud2, laser_scan_callback) 

    timer = rospy.Timer(rospy.Duration(secs=5), quit_func)
    #

    # do not exit until node is stopped
    rospy.spin()
    #try:
    #    rospy.spin()
    #except KeyboardInterrupt:
    #    quit_func()


if __name__ == '__main__':
    main()