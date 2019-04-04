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

        self.promp_goal = np.array(np.ones(4))
        self.promp_goal_trina = [None, None, None]
        
        self.kinect_sub = rospy.Subscriber("/skeleton_data", skeleton, self.callback0, queue_size = 1)
        self.otp_sub = rospy.Subscriber("/otp_estimate", PointStamped, self.callback1, queue_size=1)
        self.promp_sub = rospy.Subscriber("/promp_data", Float64MultiArray, self.callback2, queue_size = 1)


    def callback0(self,data):
        self.data = data
        P_rs = np.array([self.data.joints[0].x, self.data.joints[0].y, self.data.joints[0].z])
        P_ls = np.array([self.data.joints[1].x, self.data.joints[1].y, self.data.joints[1].z])
        P_o = (P_rs + P_ls)/2
        theta = np.pi - np.arctan2((P_rs[2] - P_ls[2]),(P_rs[0] - P_ls[0]))
        self.tf_k2h = np.array([[np.cos(theta), 0, -np.sin(theta), P_o[0]], [0, 1, 0, P_o[1]], [np.sin(theta), 0, np.cos(theta), P_o[2]], [0, 0, 0, 1]])

        self.tf_h2k = np.linalg.inv(self.tf_k2h)

    def callback1(self, data):
        self.otp = data
        self.w = self.otp.header.stamp.nsecs
        self.otp_point = np.array([self.otp.point.x, self.otp.point.y, self.otp.point.z, 1])
        # self.otp_point = np.matmul(self.tf_h2k, self.otp_point)
        self.otp_goal_trina = np.array([self.otp_point[2] + 0.115, self.otp_point[0], self.otp_point[1] + 1.65])

        #print "otp-rx"

        if(self.flag == True):
            if self.promp_goal_trina[0]:
                self.goal = ((1 - self.w)*self.promp_goal_trina) + (self.w*self.otp_goal_trina) # self.promp_goal_trina
                print "otp+promp-used"
            else:
                self.goal = self.otp_goal_trina
                print "waiting for p"

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

        self.promp_goal = np.matmul(self.tf_h2k, self.promp_goal)
        # self.promp_goal_trina = np.array([self.promp_goal[2] + 0.115 , self.promp_goal[0] , self.promp_goal[1] + 1.65])
        self.promp_goal_trina = np.array([self.promp_goal[2] + 0.22 , self.promp_goal[0] , self.promp_goal[1] + 1.65])


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