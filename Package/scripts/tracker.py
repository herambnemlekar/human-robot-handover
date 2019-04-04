#!/usr/bin/env python
import sys
import rospy
import numpy as np
from handover.msg import skeleton
import baxter_interface
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
from pyquaternion import Quaternion
from handover.msg import skeleton
import time
import scipy.io as sio

class Tracker:

    def __init__(self):

        #Publish human wrist data
        self.wrist_pub = rospy.Publisher("wrist_data", PointStamped, queue_size = 10)  
        #Publish baxter joint angles
        self.baxter_pub = rospy.Publisher("baxter_joints", Float64MultiArray, queue_size = 10)      

        #Subscribing to the skeleton data from kinect
        self.data_sub = rospy.Subscriber("skeleton_data", skeleton, self.callback, queue_size = 10)

         # Initialize
        self.i = 0
        self.demo_baxter_joints = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])  #Single demonstration
        self.demo_baxter_pos = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.demo_human_wrist = np.matrix([0.0, 0.0, 0.0])
        self.limb = baxter_interface.Limb('right')

        init_angles = {'right_s0': -1.6908303234466973, 'right_s1': 0.7175195135334023, 'right_w0': 0.060975736318445196, 'right_w1': 0.14841264122791378, 'right_w2': -0.2730485802436036, 'right_e0': 0.5909660985328556, 'right_e1': 0.9844321706254643}

        self.limb.move_to_joint_positions(init_angles)
        

        # Start Recording
        text = raw_input("Start recording demo? (Y/n)")
        if text  == 'n':
            print "No demo recorded"
        else:
            self.start_time = time.time()
            self.rec()


    def callback(self,data):
        self.data = data

        self.h = PointStamped()
        self.h.header.stamp.secs = float(self.data.joints[2].stamp)
        self.h.point.x = self.data.joints[2].x
        self.h.point.y = self.data.joints[2].y
        self.h.point.z = self.data.joints[2].z

        limb = baxter_interface.Limb('right')
        baxarm_angles = limb.joint_angles()
        baxarm_end = limb.endpoint_pose()

        bax_end_pos = baxarm_end['position']
        bax_end_rot = baxarm_end['orientation']
        self.b = Float64MultiArray()
        self.b.data = np.array([bax_end_pos.x, bax_end_pos.y, bax_end_pos.z, bax_end_rot.x, bax_end_rot.y, bax_end_rot.z, bax_end_rot.w])

        # self.b = Float64MultiArray()
        # self.b.data = np.array([baxarm_angles['right_s0'], baxarm_angles['right_s1'], baxarm_angles['right_e0'], baxarm_angles['right_e1'], baxarm_angles['right_w0'], baxarm_angles['right_w1'], baxarm_angles['right_w2']])
        
        P_rs = np.array([self.data.joints[0].x, self.data.joints[0].y, self.data.joints[0].z])
        P_ls = np.array([self.data.joints[1].x, self.data.joints[1].y, self.data.joints[1].z])
        P_o = (P_rs + P_ls)/2
        theta = np.pi - np.arctan2((P_rs[2] - P_ls[2]),(P_rs[0] - P_ls[0]))

        self.tf_k2h = np.array([[np.cos(theta), 0, -np.sin(theta), P_o[0]], [0, 1, 0, P_o[1]], [np.sin(theta), 0, np.cos(theta), P_o[2]], [0, 0, 0, 1]])

        self.wrist_pub.publish(self.h)
        self.baxter_pub.publish(self.b)

    def baxter_to_record(self):
        quat = Quaternion(self.b.data[6], self.b.data[3], self.b.data[4], self.b.data[5])
        Q1 = quat.transformation_matrix
        Q1[0:3,3] = [self.b.data[0], self.b.data[1], self.b.data[2]]

        tf2 = np.array([[0, 1, 0, 0], [0, 0, 1, -0.55], [1, 0, 0, -0.21], [0, 0, 0, 1]])
        B_e = np.matmul(self.tf_k2h,np.matmul(tf2,Q1))
        self.b.data[0:3] = B_e[0:3,3]

        R = B_e[0:3,0:3]
        Q2 = Quaternion(matrix=R)
        self.b.data[4:8] = Q2
        
        return self.b.data

    def human_to_record(self):
        P_rw = np.array([self.data.joints[2].x, self.data.joints[2].y, self.data.joints[2].z, 1])
        
        P_rw = np.matmul(self.tf_k2h,P_rw)

        return np.matrix(P_rw[0:3])


    def rec(self):
        while True:
            try:
                baxarm_pos = self.baxter_to_record()
                #self.demo_baxter_joints = np.concatenate((self.demo_baxter_joints, [baxrarm_data]), axis=0)

                h = self.human_to_record()
                self.demo_human_wrist = np.concatenate((self.demo_human_wrist,h), axis=0)

                self.demo_baxter_pos = np.concatenate((self.demo_baxter_pos, [baxarm_pos]), axis=0)

            except KeyboardInterrupt:
                self.i = self.i + 1
                break

        self.timer = time.time() - self.start_time

        if self.i == 1:
            self.ndemos_baxter_joints = [self.demo_baxter_joints[1:,:]]
            self.ndemos_human_wrist = [self.demo_human_wrist[1:,:]]
            self.ndemos_baxter_pos = [self.demo_baxter_pos[1:,:]]
            self.ndemos_time = [self.timer]
        else:
            self.ndemos_baxter_joints.append(self.demo_baxter_joints[1:,:])
            self.ndemos_human_wrist.append(self.demo_human_wrist[1:,:])
            self.ndemos_baxter_pos.append(self.demo_baxter_pos[1:,:])
            self.ndemos_time.append(self.timer)

        time.sleep(1)
        init_angles = {'right_s0': -1.6908303234466973, 'right_s1': 0.7175195135334023, 'right_w0': 0.060975736318445196, 'right_w1': 0.14841264122791378, 'right_w2': -0.2730485802436036, 'right_e0': 0.5909660985328556, 'right_e1': 0.9844321706254643}
        self.limb.move_to_joint_positions(init_angles)

        text = raw_input("\nRecord another demo? (Y/n)")
        if text == 'n':
            print "Only", self.i, "demo(s) recorded"
        else:
            self.demo_baxter_joints = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            self.demo_baxter_pos = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            self.demo_human_wrist = np.matrix([0.0, 0.0, 0.0])
            self.start_time = time.time()
            self.rec()

        print len(self.ndemos_baxter_pos)
        print self.ndemos_baxter_pos[0].shape
        sio.savemat('src/handover/promp/Data/demo_baxter_joints.mat',{'baxter_demo_joints_data':self.ndemos_baxter_joints})
        sio.savemat('src/handover/promp/Data/demo_baxter_pos.mat',{'baxter_demo_pos_data':self.ndemos_baxter_pos})
        sio.savemat('src/handover/promp/Data/demo_time.mat',{'time':self.ndemos_time})
        sio.savemat('src/handover/promp/Data/demo_human.mat',{'human_demo_data':self.ndemos_human_wrist})


def main(args):

    rospy.init_node('Tracker', anonymous=True, disable_signals=True)
    trac = Tracker()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
