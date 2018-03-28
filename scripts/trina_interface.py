#!/usr/bin/env python
import rospy
import sys, time, os
import numpy as np
from handover.msg import skeleton
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
# from baxter_pykdl import baxter_kinematics

class MoveTrina:
    
    def __init__(self):
        self.flag = rospy.get_param('otp')

        self.goal_pub = rospy.Publisher("/goal_location", PointStamped, queue_size=1)

        # self.limb = baxter_interface.Limb('right')
        # self.init_angles = self.limb.joint_angles()
        # self.kin = baxter_kinematics('right')

        self.promp_goal = np.array(np.zeros(3))
        self.promp_goal_trina = [None, None, None]
        
        self.otp_sub = rospy.Subscriber("/otp_estimate", PointStamped, self.callback1, queue_size=1)
        self.promp_sub = rospy.Subscriber("/promp_data", Float64MultiArray, self.callback2, queue_size = 1)

    def callback1(self, data):
        self.otp = data
        self.w = self.otp.header.stamp.nsecs
        self.otp_goal_trina = np.array([self.otp.point.z + 0.115, self.otp.point.x, self.otp.point.y + 1.65])

        #print "otp-rx"

        if(self.flag == True):
            if self.promp_goal_trina[0]:
                self.goal = ((1 - self.w)*self.promp_goal_trina) + (self.w*self.otp_goal_trina) # self.promp_goal_trina
                # print "otp+promp-used"
            else:
                self.goal = self.otp_goal_trina
                # print "waiting for p"

            self.goal_location = PointStamped()
            self.goal_location.point.x = self.goal[0]
            self.goal_location.point.y = self.goal[1]
            self.goal_location.point.z = self.goal[2]

            self.goal_pub.publish(self.goal_location)

        else:
            if self.promp_goal_trina[0]:
                self.goal = self.promp_goal_trina

                self.goal_location = PointStamped()
                self.goal_location.point.x = self.goal[0]
                self.goal_location.point.y = self.goal[1]
                self.goal_location.point.z = self.goal[2]

                self.goal_pub.publish(self.goal_location)

    def callback2(self, data):
        self.promp = data

        # for i in range(len(self.init_angles)):
        #     self.angles[i] = promp.data[i*99]
    
        # self.angles = self.changeByAngles(self.angles, self.init_angles)
        # self.promp_goal = self.kin.forward_position_kinematics(self.angles)

        for i in range(3):
            self.promp_goal[i] = self.promp.data[((i+1)*99) - 1]

        # self.promp_goal_trina = np.array([self.promp_goal[0], self.promp_goal[1], self.promp_goal[2] + 87.5])
        self.promp_goal_trina = np.array([self.promp_goal[2] + 0.18 , self.promp_goal[0] , self.promp_goal[1] + 1.65])

        #print "promp-rx"

    def changeByAngles(self,change_angles, angles):
        angles['right_s0']=change_angles[0]
        angles['right_s1']=change_angles[1]

        angles['right_e0']=change_angles[2]
        angles['right_e1']=change_angles[3]

        angles['right_w0']=change_angles[4]
        angles['right_w1']=change_angles[5]
        angles['right_w2']=change_angles[6]

        return angles

    # def move(self, position):
    #     goal_angles = self.kin.inverse_kinematics(position, self.promp_goal[3:8])
    #     self.limb.set_joint_positions(goal_angles)

def main(args):  
    rospy.init_node('MoveTrina', anonymous=True)
    mt = MoveTrina()   
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
