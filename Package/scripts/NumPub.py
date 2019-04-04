#!/usr/bin/env python
import roslib
roslib.load_manifest('handover')
import rospy
import sys, time, os
import numpy as np
from geometry_msgs.msg import PointStamped


class NumPub:


    def __init__(self):
        self.num_pub = rospy.Publisher("/hello", PointStamped, queue_size=1)
        Z = PointStamped()
        Z.point.x = 0.4
        Z.point.y = 0.0
        Z.point.z = 1.35
        r = rospy.Rate(10) 
        while not rospy.is_shutdown():
            print "publishing"
            self.num_pub.publish(Z)
            r.sleep()
					
def main(args): 
    rospy.init_node('NumPub', anonymous=True)
    np = NumPub()	
    try:
	rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
