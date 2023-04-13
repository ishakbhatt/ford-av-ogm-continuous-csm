import pickle
import os
import numpy as np

data_directory = "./data/"

with open(os.path.join(data_directory, "data_aligned.pkl"), "rb") as f:
    data = pickle.load(f)

max_x = data["laserScan"][0][0][0][0]
max_y = data["laserScan"][0][0][1][0]
min_x = data["laserScan"][0][0][0][0]
min_y = data["laserScan"][0][0][1][0]

for i in range(len(data["laserScan"])):
    local_max_x = max(data["laserScan"][i][0][0])
    local_max_y = max(data["laserScan"][i][0][1])
    local_min_x = min(data["laserScan"][i][0][0])
    local_min_y = min(data["laserScan"][i][0][1])
    #print(local_max_x, local_max_y, local_min_x, local_min_y)
    max_x = max(max_x, local_max_x)
    max_y = max(max_y, local_max_y)
    min_x = min(min_x, local_min_x)
    min_y = min(min_y, local_min_y)

print(min_x, max_x, min_y, max_y)

max_pose_x = max(data["robotPose"]["x"][0][0])
max_pose_y = max(data["robotPose"]["y"][0][0])
min_pose_x = min(data["robotPose"]["x"][0][0])
min_pose_y = min(data["robotPose"]["y"][0][0])

print(min_pose_x, max_pose_x, min_pose_y, max_pose_y)