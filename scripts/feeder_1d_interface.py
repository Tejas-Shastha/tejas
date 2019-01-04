#!/usr/bin/env python

import rospy
import sys
import pandas as pd
import numpy as np
from hri_package.msg import Sens_Force
from std_msgs.msg import Int32

FORCE_VALUES = Sens_Force()
ARM_STATE = Int32()

def forceListener(data):
    global FORCE_VALUES
    FORCE_VALUES = data

def armListener(data):
    global ARM_STATE
    ARM_STATE = data

def main():
    rospy.init_node("feeder_1d_interface")
    rospy.Subscriber("/force_values", Sens_Force, forceListener)
    rospy.Subscriber("/arm_state", Int32, armListener)
    rate = rospy.Rate(10)
        
    print("CSV location: ", sys.argv[1])
    reward_frame = pd.read_csv(sys.argv[1])
    reward_table = reward_frame.values
    print(reward_frame)
    print("Reward for S3, A2 : {}".format(reward_table[2,1]))


    while not rospy.is_shutdown():
        #print("Received F: {} B: {} Arm: {}".format(FORCE_VALUES.forceF, FORCE_VALUES.forceB, ARM_STATE))
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
