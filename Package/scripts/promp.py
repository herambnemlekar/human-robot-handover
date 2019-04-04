#!/usr/bin/env python
import sys
import rospy
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped

class ProMP:
    
    def __init__(self):

        self.goal_pub = rospy.Publisher("/promp_data", Float64MultiArray, queue_size=1)

        self.dt = 0.01
        self.std_dev = 

        self.promp_sub = rospy.Subscriber("/wrist_data", PointStamped, self.callback, queue_size = 1)

    def callback(self, data):

    def load(self):

    def train(self):

    def observation(self):

    def run(self):

def main(args):  
    rospy.init_node('ProMP', anonymous=True)
    pmp = ProMP()   

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)