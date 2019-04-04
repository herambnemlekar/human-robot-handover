#!/usr/bin/env python
import rospy
import sys, time, os
import numpy as np
from handover.msg import skeleton
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
import baxter_interface
import csv

class OtpGenerator:


    def __init__(self):
        
        #Publishing the estimated goal position and total trajectory time
        self.goal_pub = rospy.Publisher("/otp_estimate", PointStamped, queue_size=1)
        self.wrist_pub = rospy.Publisher("/wrist_data", PointStamped, queue_size=1)
        
        self.D = np.matrix([0, 0, 0, 0])    #For Storing the Kinect sensor data         
        self.offset = 0.1   #Hacked offset distance from the wrist to object
        self.t_demo = 1.5   #Total time of the demonstration
        self.baxter = np.matrix([[-0.5,0.0,0.25],[-0.5,-0.9,0.25],[0.5,0.0,0.25],[0.5,-0.9,0.25]])  #Baxter arm length 
        self.w = 1.0    #Weight factor for determining the confidence in the dynamic OTP
        self.i = 0.0    #Initialization for otp loop
        self.gain = 0.75  #Gain for adjusting weights
        self.duration = 0.0
        self.Z = None   

        #Subscribing to the skeleton data from kinect
        self.data_sub = rospy.Subscriber("skeleton_data", skeleton, self.static, queue_size = 1)   

        self.previous_baxarm_pos = np.array([0., 0., 0., 1.])
        
    def static(self,data):

        self.data = data
    
        #Sorting skeleton data
        P_rs = np.matrix([self.data.joints[0].x, self.data.joints[0].y, self.data.joints[0].z, float(self.data.joints[0].stamp)])
        self.P_rw = np.matrix([self.data.joints[2].x, self.data.joints[2].y, self.data.joints[2].z, float(self.data.joints[2].stamp)])
        P_ls = np.matrix([self.data.joints[1].x, self.data.joints[1].y, self.data.joints[1].z, float(self.data.joints[1].stamp)])
        P_lw = np.matrix([self.data.joints[3].x, self.data.joints[3].y, self.data.joints[3].z, float(self.data.joints[3].stamp)])
        
        #Formatting the sensor data
        self.D = np.append(self.D, self.P_rw, axis=0)
        self.D = self.D[(np.shape(self.D)[0] - 2):np.shape(self.D)[0],:]

        if self.i==0:
            #Extracting wrist and shoulder positions of human and baxter
            P1 = np.true_divide((P_rs[0,0:3] + self.baxter[2,0:3]), 2.0)    #Right Shoulder of human & Left Shoulder of Baxter
            P2 = np.true_divide((P_ls[0,0:3] + self.baxter[0,0:3]), 2.0)    #Left Shoulder of human & Right Shoulder of Baxter
            P3 = np.true_divide((self.P_rw[0,0:3] + self.baxter[3,0:3]), 2.0)   #Right Wrist of human & Left Wrist of Baxter
            P4 = np.true_divide((P_lw[0,0:3] + self.baxter[1,0:3]), 2.0)    #Left Wrist of human & Right Wrist of Baxter
        
            self.otp_s = np.true_divide((P1 + P2 + P3 + P4), 4.0) #OTP static calculation 
            
            print(self.otp_s)
        
        self.i = self.i + 1
                
        if self.i>1:
            self.dynamic()

    
    def dynamic(self):

        #Object position
        self.P_obj = np.add(self.P_rw[0,0:3], [0, self.offset, 0])
        
        #Human wrist position: current and previous
        self.P_new = self.D[1, 0:3]
        self.P_old = self.D[0, 0:3]
        
        #Distance between current location and goal position
        self.e_old = np.linalg.norm(self.P_old - self.otp_s)
        self.e_new = np.linalg.norm(self.P_new - self.otp_s)
            
        if self.w>0.005:
            #Total time of the demonstration
            self.t_goal = self.t_demo   
        
            if (self.e_old - self.e_new) > 0.003:   #Arbitrary threshold for detecting movement


                if self.w == 1:
                    self.start_time = time.time()

                #Time step
                dt = self.D[1, 3] - self.D[0, 3]
            
                #Calculating weights
                self.w = self.w - ((dt/self.t_goal)*self.gain)   #Arbitrary gain for reducing the weight
                print self.w
            
                #Weighted average to find the goal
                self.P_goal = (self.w*self.otp_s) + ((1-self.w)*self.P_obj)
                
                #Instantaneous velocity
                V = (self.P_new - self.P_old)/dt
                V_dot = np.dot(V, (self.P_new - self.otp_s).T)
                
                #Estimating total time of the trajectory
                self.t_goal = np.linalg.norm(self.P_goal - self.P_obj)/V_dot
            
                #Publishing the goal position and estimated time (to the next block -DMP)
                otp_data = np.array([self.P_goal[0,0],self.P_goal[0,1],self.P_goal[0,2],self.t_goal])


                self.Z = PointStamped()
                self.Z.header.stamp.secs = self.P_rw[0,3]
                self.Z.header.stamp.secs = self.w
                self.Z.point.x = self.P_goal[0,0]
                self.Z.point.y = self.P_goal[0,1]
                self.Z.point.z = self.P_goal[0,2]

                #self.goal_pub.publish(self.Z)
                
                #with open('goaldata2.txt','a') as file:
                #   file.write(str(self.P_goal)+"\n")
                                
            
            if self.w<0.25:
                self.P_goal = self.P_obj
                
                otp_data = np.array([self.P_goal[0,0],self.P_goal[0,1],self.P_goal[0,2],self.t_goal])

                self.Z = PointStamped()
                self.Z.header.stamp.secs = self.P_rw[0,3]
                self.Z.header.stamp.secs = self.w
                self.Z.point.x = self.P_goal[0,0]
                self.Z.point.y = self.P_goal[0,1]
                self.Z.point.z = self.P_goal[0,2]

            if self.Z:
                self.goal_pub.publish(self.Z)
                
                #print("else",self.P_goal)

                #with open('goaldata2.txt','a') as file:
                #   file.write(str(self.P_goal)+"\n")

        self.P = PointStamped()
        self.P.header.stamp.secs = self.P_rw[0,3]
        self.P.header.stamp.nsecs = (self.w*100)
        if self.w<0:
            self.P.header.stamp.nsecs = 0
        self.P.header.frame_id = str(self.P.header.stamp.nsecs)
        self.P.point.x = self.P_rw[0,0]
        self.P.point.y = self.P_rw[0,1]
        self.P.point.z = self.P_rw[0,2]

        self.wrist_pub.publish(self.P)

        limb = baxter_interface.Limb('right')
        baxarm_end = limb.endpoint_pose()
        baxarm_pos = baxarm_end['position']
        baxarm_pos = np.array([baxarm_pos.x, baxarm_pos.y, baxarm_pos.z, 1])
        tf = np.array([[0, 1, 0, 0], [0, 0, 1, -0.55], [1, 0, 0, -0.21], [0, 0, 0, 1]])
        baxarm_pos = np.matmul(tf,baxarm_pos)


        if self.w < 0.1 and (self.e_old - self.e_new) < 0.0025 and ((np.linalg.norm(baxarm_pos[0:3] - self.previous_baxarm_pos[0:3])) < 0.0005) and self.duration==0:
            time_now = time.time()
            self.duration = time_now - self.start_time
            self.error = np.linalg.norm(self.P_rw[0,0:3] - baxarm_pos[0:3])
            print "will now print"
            # print self.P_goal
            # print self.P_obj
            # print self.otp_s
            csv_file  = open('Experiment_Data.csv', 'a')
            csv_file.write(str(self.duration) + "," + str(self.error) + "\n")
        
        self.previous_baxarm_pos = baxarm_pos
                
        
def main(args):  
    rospy.init_node('OtpGenerator', anonymous=True)
    og = OtpGenerator() 
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
