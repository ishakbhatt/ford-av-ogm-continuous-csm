import velodyne_decoder as vd
import numpy as np

config = vd.Config(model='VLP-32C')
bagfile = '/home/coned/data/Sample-Data.bag'
lidar_topics = ['/lidar_blue_scan', '/lidar_green_scan']
cloud_arrays = []
for stamp, points, topic in vd.read_bag(bagfile, config, lidar_topics):
    cloud_arrays.append(points)
    #print(type(points.dtype))

print((cloud_arrays[0][0][0]))