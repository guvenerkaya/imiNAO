#!/usr/bin/env python2.7
import rospy
import time
import sys
import motion
import numpy as np
import cv2
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import JointState
from control_v2.srv import MoveJoints
from cv_bridge import CvBridge, CvBridgeError
from naoqi import ALProxy

# set parameter on start
motionProxy = 0



class Control:

    def __init__(self):
        # init class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.stiffness = False  
        self.key = ""
        
        pass

    def keyboard_data(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.key = data.data

    def joints_data(self,data):
        # rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name
        self.joint_angles = data.position
        self.joint_velocities = data.velocity

        pass

    # sets the stiffness for all joints. can be refined to only toggle single joints, set values between [0,1] etc
    def set_stiffness(self,value):
        if value == True:
            service_name = '/body_stiffness/enable'
        elif value == False:
            service_name = '/body_stiffness/disable'
        try:
            stiffness_service = rospy.ServiceProxy(service_name,Empty)
            stiffness_service()
        except rospy.ServiceException, e:
            rospy.logerr(e)

    def set_joint_angles(self,  Lshoulder_angle,
                                LShoulderRoll_angle, 
                                Rshoulder_angle, 
                                RShoulderRoll_angle, 
                                Head_angle, 
                                LElbowRoll_angle,
                                RElbowRoll_angle,
                                LElbowYaw_angle,
                                RElbowYaw_angle,
                                LWristYaw_angle,
                                RWristYaw_angle):

        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("LShoulderPitch") # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(Lshoulder_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.joint_names.append("LShoulderRoll")
        joint_angles_to_set.joint_angles.append(LShoulderRoll_angle)
        joint_angles_to_set.joint_names.append("RShoulderPitch")
        joint_angles_to_set.joint_angles.append(Rshoulder_angle)
        joint_angles_to_set.joint_names.append("RShoulderRoll")
        joint_angles_to_set.joint_angles.append(RShoulderRoll_angle)
        joint_angles_to_set.joint_names.append("HeadPitch")
        joint_angles_to_set.joint_angles.append(Head_angle)
        joint_angles_to_set.joint_names.append("LElbowRoll")
        joint_angles_to_set.joint_angles.append(LElbowRoll_angle)
        joint_angles_to_set.joint_names.append("RElbowRoll")
        joint_angles_to_set.joint_angles.append(RElbowRoll_angle)
        joint_angles_to_set.joint_names.append("LElbowYaw")
        joint_angles_to_set.joint_angles.append(LElbowYaw_angle)
        joint_angles_to_set.joint_names.append("RElbowYaw")
        joint_angles_to_set.joint_angles.append(RElbowYaw_angle)
        # hand joint parameters
        joint_angles_to_set.joint_names.append("LWristYaw")
        joint_angles_to_set.joint_angles.append(LWristYaw_angle)
        joint_angles_to_set.joint_names.append("RWristYaw")
        joint_angles_to_set.joint_angles.append(RWristYaw_angle)
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)

    # def get_cartesian_pos(self):
        # get positions of endeffector/joint
        # name = "LArm"
        # frame = 0
        # useSensorValues = True

        # print("requested joint: {}".format(name))
        # current = motionProxy.getPosition(name, frame, useSensorValues)
        # print("Position (meters) of {} in Torso is:\nx = {},\ny = {},\nz = {}"
        #                                             .format(name,
        #                                                     current[0],
        #                                                     current[1],
        #                                                     current[2]))
        # print("Orientation (radians) of {} in Torso is:\nroll  = {},\npitch = {},\nyaw   = {}"
        #                                             .format(name,
        #                                                     current[3], 
        #                                                     current[4], 
        #                                                     current[5]))
        # print("=====================================================")
        # obtain_pos = 1
        # return obtain_pos
        
    # def set_cartesian_pos(self):

    def central_control(self):

        # create node and env #
        robotIP=str(sys.argv[1])
        PORT=int(sys.argv[2])
        print(sys.argv[2])
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
        rospy.init_node('move_joints_server')

        # create several topic subscriber #
        rospy.Subscriber("key", String, self.keyboard_data)
        rospy.Subscriber("joint_states", JointState, self.joints_data)
        self.jointPub = rospy.Publisher("joint_angles", JointAnglesWithSpeed, queue_size=10)

        # set parameters #
        obtain_pos = 0

        # call service #
        #s = rospy.Service("move_joints_service", MoveJoints, Control.handle_move_joints)


        ## set stiffness ON: ##
        # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        self.set_stiffness(True)

        # init_posture, Zero (NOT "T-position")
        postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
        postureProxy.goToPosture("StandZero", 0.6)
        rospy.sleep(2.0)
        self.set_joint_angles(0,1.1,0,-1.1,0,0,0,0,0,0,0)
        rospy.sleep(2.0)

        
        ###########################################################################
        ## test sequence to demonstrate setting joint angles ##
        #rospy.sleep(3.0)
        #self.set_joint_angles(0.5,-0.1,0.5,0.1, 0.1,0) 
        #rospy.sleep(3.0)   
        #self.set_joint_angles(0.0)
        #rospy.sleep(3.0)

        ############################################################################
        ## get positions of endeffector/joint
        name = "LArm"
        frame = 0
        useSensorValues = True

        ###########################################################################
        ## set position ##
        effector   = name
        space      = frame
        axisMask   = 63
        isAbsolute = True
        maxSpeedFraction = 0.5
        times      = [3, 4]

        # set the current position is zero
        currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # get the changes relative to the current position 
        targetPos  = [0.05, 0.05, 0.05, 0.0, 0.0, 0.0]

        # Go to the target and back again
        path = [targetPos, currentPos]

        # if times are set use positionInterpolation otherwise use setPositions
        # if len(times) > 0:
        #     motionProxy.positionInterpolation(effector, space, path,
        #                                     axisMask, times, isAbsolute)
        # else:
        #     motionProxy.setPositions(effector, space, target, maxSpeedFraction, axisMask)
        


        while True:

            if self.key == 'h': # set the selected joints in 'Zero position'
                #self.set_joint_angles(0,0,0,0,0,0,0,0,0,0,0,0)
                postureProxy.goToPosture("StandZero", 0.5)
            elif self.key == 'i': # set the init 'T-position'
                self.set_joint_angles(0,1.1,0,-1.1,0,0,0,0,0,0,0)
                rospy.sleep(1.0)
            elif self.key == 'g': # get position
                if obtain_pos == 0:
                    print("requested joint: {}".format(name))
                    current = motionProxy.getPosition(name, frame, useSensorValues)
                    print("Position (meters) of {} in Torso is:\nx = {},\ny = {},\nz = {}"
                                                                .format(name,
                                                                        current[0],
                                                                        current[1],
                                                                        current[2]))
                    print("Orientation (radians) of {} in Torso is:\nroll  = {},\npitch = {},\nyaw   = {}"
                                                                .format(name,
                                                                        current[3], 
                                                                        current[4], 
                                                                        current[5]))
                    print("=====================================================")
                    obtain_pos = 1
                    #self.get_cartesian_pos() -> later fct.
            elif self.key == 'r':
                if obtain_pos == 1:
                    dx = targetPos[0] + current[0]
                    dy = targetPos[1] + current[1]
                    dz = targetPos[2] + current[2]
                    droll  = targetPos[3]  + current[3]
                    dpitch = targetPos[4]  + current[4]
                    dyaw   = targetPos[5]   + current[5]

                    targetPos = [dx, dy, dz, droll, dpitch, dyaw]
                    motionProxy.positionInterpolation(effector, space, path,
                                            axisMask, times, isAbsolute)
                    print("motion completed")
                    obtain_pos = 0
            elif self.key == 'a': #refresh setting para
                if obtain_pos == 1:
                    
                    motionProxy.positionInterpolation(effector, space, path,
                                            axisMask, times, isAbsolute)
                    print("motion completed")
                    obtain_pos = 0
            elif self.key == 'f': #refresh setting para
                obtain_pos = 0
                
            elif self.key == 'q':
                break

        ## set stiffness OFF: (not needed, when using motionProxy.rest ##
        # always check that your robot is in a stable position before disabling the stiffness!!
        # self.set_stiffness(False) 

        rate = rospy.Rate(10) # sets the sleep time to 10ms                    

        #rospy.sleep(3.0)
        print("motion task ended")

        # Set NAO to resting position
        motionProxy.rest()
        print("NAO resting")
        time.sleep(2.0)
        rospy.spin()
        
        while not rospy.is_shutdown():
            self.set_stiffness(self.stiffness)
            rate.sleep()


    # service handler
    def handle_move_joints(request):

        err_code = 0
            
        # Set NAO to start-up position
        # motionProxy.wakeUp()
        # time.sleep(2.0)

        # Set Nao Stiffness ON
        motionProxy.setStiffnesses("Body", 1.0)

        # init_posture
        # if request.init_posture:
        postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
        postureProxy.goToPosture("StandZero", 0.5)
        rospy.sleep(3.0)
        print("NAO in init position, ready to move")
        #return err_code

        # effector   = request.name
        # space      = request.frame
        # axisMask   = request.axisMask
        # isAbsolute = request.absolute
        # maxSpeedFraction = request.maxSpeedFraction
        # times      = request.times

        # set the current position is zero
        # currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # get the changes relative to the current position
        # targetPos  = request.target

        # Go to the target and back again
        # path = [targetPos, currentPos]

        # if times are set use positionInterpolation otherwise use setPositions
        # if len(times) > 0:
        #     motionProxy.positionInterpolation(effector, space, path,
        #                                     axisMask, times, isAbsolute)
        # else:
        #     motionProxy.setPositions(effector, space, target, maxSpeedFraction, axisMask)
        
        rospy.sleep(3.0)

        print("motion completed")

        # Set NAO to resting position
        motionProxy.rest()
        print("NAO resting")

        # Set Nao Stiffness OFF
        #motionProxy.setStiffnesses("Body", 0.0)
        time.sleep(2.0)
        print("motion completed")
        return err_code



if __name__ == '__main__':
    try:
        central_instance = Control()
        central_instance.central_control()

        #rospy.spin()

    except rospy.ROSInterruptException:
        pass