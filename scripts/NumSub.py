#!/usr/bin/env python
import roslib
roslib.load_manifest('handover')
import rospy
import sys, time, os
import numpy as np
# from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped

class NumSub:


    def __init__(self):	
        self.num_sub = rospy.Subscriber("/goal_location", PointStamped, self.callback, queue_size = 1)	
		
    def callback(self,data):
	# self.data = data
	print(data.point.x)
					
def main(args):  
    rospy.init_node('NumSub', anonymous=True)
    ns = NumSub()	
    try:
	rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
