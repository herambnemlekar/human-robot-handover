'''
    A Task Generator for forwarding motion commands into the TRINA stack.
    Based heavily (copied) from Kris hauser's Oct 2015 game_pad.py TaskGenerator.

    Converted by Gunnar Horve and Max Merlin Jan 2015.

    Subscribes to a robot OTP handover topic.  Aimed at being used for autonomous
    robotic handovers.
'''
# Import Modules
import os
import sys
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from task_generator import TaskGenerator

from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase
import rospy
# from otp.msg import pos
from geometry_msgs.msg import PointStamped

class MyWidgetPlugin(GLPluginBase):
    ''' Hook class to allow event passing to/from Klampt && OpenGL UI '''
    def __init__(self,taskGen): #taskGen is an instance ofSkittlesTaskGenerator
        GLPluginBase.__init__(self)
        self.taskGen = taskGen

    def initialize(self):
        GLPluginBase.initialize(self)
        return True

    def keyboardfunc(self,c,x,y):
        if(c == 'g'):
            self.taskGen.time_to_grab = True
        print("Someone typed " + c)

    def display(self):
        pass

    def display_screen(self):
        pass

    def eventfunc(self,type,args):
        print("we got an EVENT!")

class SkittlesTaskGenerator(TaskGenerator):
    '''
        Logic class for sending commands to TRINA.
        See TaskGenerator for method definitions
    '''

    def __init__(self):
        self.limb = 'right'
        self.lastState = {}
        self.plugin = None
        self.time_to_grab = False

        # set up OTP subscriber && defalt values
        rospy.Subscriber("/goal_location", PointStamped, self.updateOTP)
        self._x = 0.5
        self._y = -0.5
        self._z = 0.75

    def name(self): return "GSkittles"

    def init(self,world):
        self.world = world
        return True

    def start(self):
        self.limb = 'right'
        self._status = 'ok'
        self.plugin = MyWidgetPlugin(self)
        self.lastState = {}
        return True

    def status(self):
        return 'ok'

    def messages(self):
        return ["Controlling "+self.limb]

    def controlMode(self):
        return "Cartesian position"

    def stop(self):
        self._status=''
        self.plugin = None

    def get(self):
        #+x is forward, +z is up, +y is left
        #For details on the args go to cartesian_pose.py in controller/tasks
        if(self.time_to_grab):
            self.time_to_grab = False
            #return "grab message"
            gripMsg = {"type":"Gripper",
                       "limb":"right",
                       "force":0.4,
                       "speed":0.2,
                       "position":[0.2,0.2,0.2] + [1.0]}
            return gripMsg

        pos_msg =  {"type":"CartesianPose",
                    "limb":"right", "position":[self._x,self._y,self._z],
                    "rotation":[1,0,0,0,1,0,0,0,1],
                    "speed":1,
                    "maxJointDeviation":100,
                    "safe":0}
        return pos_msg

    def glPlugin(self):
        return self.plugin

    def updateOTP(self, data):
        # self._x = data.point.z + 0.2
        # self._y = data.point.x
        # self._z = data.point.y + 1.365

        self._x = data.point.x
        self._y = data.point.y
        self._z = data.point.z

        print self._x

def make():
    ''' Hook function that C++ aspects of Ebolabot stack grab. '''
    print "make called"
    return SkittlesTaskGenerator()
