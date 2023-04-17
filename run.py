# Main script to construct map
#
# Author: Isha Bhatt, Meghana Kowsika, Yipeng Lin, Yanjun Chen, Thirumalaesh Ashokkumar
# Date: 04/19/2023

import pickle
import os
from utils import plot_mean, plot_variance
from ogm_continuous_CSM import ogm_continuous_CSM
sys.path.append('.')
import yaml
with open("config/settings.yaml", 'r') as stream:
    param = yaml.safe_load(stream)

def main():

    data_directory = param['data_directory']
    with open(os.path.join(data_directory, "data_aligned.pkl"), "rb") as f:
        data_aligned = pickle.load(f)

    robotPose = data_aligned['robotPose']
    laserScan = data_aligned['laserScan']

    # continuous S-CSM
    ogm = ogm_continuous_CSM()
        
    # build map
    ogm.construct_map(robotPose, laserScan)
    ogm.build_ogm()

    # plot
    plot_mean(ogm, 'CSM Mean', 'ogm_intel_CSM_mean.png')
    plot_variance(ogm, 'CSM Variance', 'ogm_intel_CSM_variance.png')


if __name__ == '__main__':
    main()
