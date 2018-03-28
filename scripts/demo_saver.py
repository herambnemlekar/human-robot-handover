#!/usr/bin/env python
import sys
import rospy
import numpy as np
import baxter_interface
import scipy.io as sio
from handover.msg import skeleton
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
import time


class Saver:

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

        baxarm_angles = self.limb.joint_angles()

        self.b = Float64MultiArray()
        self.b.data = np.array([baxarm_angles['right_s0'], baxarm_angles['right_s1'], baxarm_angles['right_e0'], baxarm_angles['right_e1'], baxarm_angles['right_w0'], baxarm_angles['right_w1'], baxarm_angles['right_w2']])

        baxarm_pos = self.limb.endpoint_pose()
        self.baxarm_pos = np.array([baxarm_pos['position'].x, baxarm_pos['position'].y, baxarm_pos['position'].z, baxarm_pos['orientation'].x, baxarm_pos['orientation'].y, baxarm_pos['orientation'].z, baxarm_pos['orientation'].w])
        
        self.wrist_pub.publish(self.h)
        self.baxter_pub.publish(self.b)
        

    def baxter_to_record(self):
        return self.b.data      

    def human_to_record(self):
        self.P_rw = np.matrix([self.data.joints[2].x, self.data.joints[2].y, self.data.joints[2].z])
        return self.P_rw


    def rec(self):
        while True:
            try:
                baxrarm_data = self.baxter_to_record()
                self.demo_baxter_joints = np.concatenate((self.demo_baxter_joints, [baxrarm_data]), axis=0)

                h = self.human_to_record()
                self.demo_human_wrist = np.concatenate((self.demo_human_wrist,h), axis=0)

                self.demo_baxter_pos = np.concatenate((self.demo_baxter_pos, [self.baxarm_pos]), axis=0)

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

        sio.savemat('src/handover/promp/Data/demo_baxter_joints.mat',{'baxter_demo_joints_data':self.ndemos_baxter_joints})
        sio.savemat('src/handover/promp/Data/demo_baxter_pos.mat',{'baxter_demo_pos_data':self.ndemos_baxter_pos})
        sio.savemat('src/handover/promp/Data/demo_time.mat',{'time':self.ndemos_time})
        sio.savemat('src/handover/promp/Data/demo_human.mat',{'human_demo_data':self.ndemos_human_wrist})

def main(args):
    rospy.init_node('Saver', anonymous=True, disable_signals=True)
    sav = Saver()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)