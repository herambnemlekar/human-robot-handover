#!/usr/bin/env python
import sys
import rospy
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
import baxter_interface
from baxter_pykdl import baxter_kinematics

class MoveBaxter:
    
    def __init__(self):

        self.limb = baxter_interface.Limb('right')
        self.init_angles = self.limb.joint_angles()

        # self.kin = baxter_kinematics('right')

        self.baxarm_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Initial baxter's human-like pose:
        # init_angles = {'right_s0': -1.6908303234466973, 'right_s1': 0.7175195135334023, 'right_w0': 0.060975736318445196, 'right_w1': 0.14841264122791378, 'right_w2': -0.2730485802436036, 'right_e0': 0.5909660985328556, 'right_e1': 0.9844321706254643}
        # self.limb.move_to_joint_positions(init_angles)

        self.num_sub = rospy.Subscriber("/promp_data", Float64MultiArray, self.callback, queue_size = 1)

    def callback(self, data):
        promp = data

        for i in range(len(self.baxarm_data)):
            self.baxarm_data[i] = promp.data[((i+1)*99)-1]
    
        self.joint_angles = self.changeByAngles(self.baxarm_data, self.init_angles)
        self.move_joints(self.limb,self.joint_angles)

        # position = self.baxarm_data[0:3]
        # rotation = self.baxarm_data[3:8]
        # self.move_end(position,rotation)

    def changeByAngles(self,change_angles, angles):
        angles['right_s0']=change_angles[0]
        angles['right_s1']=change_angles[1]

        angles['right_e0']=change_angles[2]
        angles['right_e1']=change_angles[3]

        angles['right_w0']=change_angles[4]
        angles['right_w1']=change_angles[5]
        angles['right_w2']=change_angles[6]

        return angles

    def move_joints(self,limb,angles):
        # limb.move_to_joint_positions(angles)
        limb.set_joint_positions(angles)

    def move_end(self,pos,rot):
        seed = [-0.09702428483375242, 0.26039323874354897, 1.4055098969000104, 1.4672526236123982, 0.794218552927673, 0.786548648988246, 0.3746748074410123]
        angles =  self.kin.inverse_kinematics(pos,rot,seed)
        print angles


def main(args):  
    rospy.init_node('MoveBaxter', anonymous=True)
    mb = MoveBaxter()   

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)




