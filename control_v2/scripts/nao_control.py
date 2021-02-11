#!/usr/bin/env python2.7
# Team C: Yassine El Himer, Gueven Erkaya, Patrick Hinz
import rospy
import time
import sys
import motion
import numpy as np
import cv2
import almath
import math
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import JointState
from perceptor.msg import Poses, Pose, Keypoint
#from control_v2.srv import MoveJoints
from cv_bridge import CvBridge, CvBridgeError
from naoqi import ALProxy

class Control:

    def __init__(self):
        # init class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.stiffness = False  
        self.key = ""
        self.poses = None
        self.depth = None

        self.robotIP=str(sys.argv[1])
        self.PORT=int(sys.argv[2])
        print(sys.argv[2])
        self.motionProxy = ALProxy("ALMotion", self.robotIP, self.PORT)
        

        # create several topic subscriber
        rospy.init_node('central_node', anonymous=True) # init node, sets name
        self.poses_sub = rospy.Subscriber("/perceptor/poses", Poses, self.poses_data) # for pose states
        self.depth_sub = rospy.Subscriber("/perceptor/depth", String, self.depth_data) # for depth state
        rospy.Subscriber("joint_states", JointState, self.joints_data) # for joint states
        self.joint_pub = rospy.Publisher("joint_angles", JointAnglesWithSpeed, queue_size=10)


        # init_posture, Zero (first "0-Pose" going to "T-position")
        postureProxy = ALProxy("ALRobotPosture", self.robotIP, self.PORT)
        postureProxy.goToPosture("StandZero", 0.6)
        rospy.sleep(0.5)
        self.set_joint_angles(0,1.1,0,-1.1,0,0,0,0,0,0,0,0,0)
        rospy.sleep(0.5)


    #get depth data from publisher
    def depth_data(self, data):
        self.depth = float(data.data)
    
    # react on keyboard input
    def keyboard_data(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.key = data.data

    # store current joint information in class variables
    def joints_data(self,data):
        # rospy.loginfo("joint states "+str(data.name)+str(data.position))
        self.joint_names = data.name
        self.joint_angles = data.position
        self.joint_velocities = data.velocity

    def depth_data_calib(self,depth):
        '''
        depricated
        # calibration of depth in current pose; here especially for arm
        # current_depth = depth # current depth 'x'
        # min_depth,max_depth = 80,250 # max and min from calibration (90 deg up and 0 deg straight to camera)
        # angle_right_elbow_yaw = -90/(max_depth-min_depth)*(current_depth-min_depth) + 90
        '''
        bound = 100
        if depth <= bound:
            angle_right_elbow_yaw = 0
            angle_right_shoulder_roll = -63
        else:
            angle_right_elbow_yaw = 90
            angle_right_shoulder_roll = 0
        
        #angle_right_elbow_yaw = self.depth_orient(angle_right_elbow_yaw)
        #angle_right_shoulder_roll = self.depth_orient(angle_right_shoulder_roll)

        print("depth: {}, r elbow yaw: {} ".format(self.depth, angle_right_elbow_yaw))
        print("depth: {}, r shoulder roll: {} ".format(self.depth, angle_right_shoulder_roll))
        
        return self.degree_to_rad_convertion([angle_right_elbow_yaw, angle_right_shoulder_roll])
    '''
    depricated
    def depth_orient(self, angle):
        points = []
        poses = self.poses
        for keypoint in poses[0].keypoints:
                if keypoint.part == keypoint.part == "rightElbow" or keypoint.part == "rightWrist":
                    x,y = keypoint.position.x, keypoint.position.y 
                    points.append((x,y))
        p1=points[0]
        p2=points[1]
        p12 = (p1[0] - p2[0]) + (p1[1] - p2[1])
        if p12 < 0:
            return -angle
        else:
            return angle
    '''

    def poses_data(self, poses_msg):

        self.poses = poses_msg.poses

        # posture setting for elbow rolls
        angle_left_elbow = self.convert_poses(self.poses, "leftElbow")
        angle_right_elbow = self.convert_poses(self.poses, "rightElbow")
        # posture setting for shoulder pitches
        angle_right_shoulder = self.convert_poses(self.poses, "rightShoulder")
        angle_left_shoulder = self.convert_poses(self.poses, "leftShoulder")
        # posture depth for elbow yaw
        if self.depth != None:
            angle_right_elbow_yaw = self.depth_data_calib(self.depth)[0]
            angle_right_shoulder_roll = self.depth_data_calib(self.depth)[1]
            angle_left_elbow_yaw = -angle_right_elbow_yaw
            angle_left_shoulder_roll = -angle_right_shoulder_roll
        else:
            angle_right_elbow_yaw = 0
            angle_left_elbow_yaw = 0
            angle_right_shoulder_roll = 0
            angle_left_shoulder_roll = 0
        #angle_right_elbow = self.convert_poses_rot(self.poses, "RightLowArm")
        angle_right_knee = self.convert_poses(self.poses, "rightKnee")
    
        # put calculated joints into an array to prepare to publish
        angles =    [angle_left_shoulder, angle_left_shoulder_roll, angle_right_shoulder, angle_right_shoulder_roll] # shoulder joints
        angles +=   [-0.2, -angle_left_elbow, angle_right_elbow, angle_left_elbow_yaw, angle_right_elbow_yaw] # head & elbow joints
        angles +=   [0, 0, 0, 0] # wrist & knee joints
        angles = self.joint_clamp(angles) # check the limits (and adjust)

        self.set_joint_angles(Lshoulder_angle       = angles[0],
                              LShoulderRoll_angle   = angles[1], 
                              Rshoulder_angle       = angles[2], 
                              RShoulderRoll_angle   = angles[3], 
                              Head_angle            = angles[4], 
                              LElbowRoll_angle      = angles[5],
                              RElbowRoll_angle      = angles[6],
                              LElbowYaw_angle       = angles[7],
                              RElbowYaw_angle       = angles[8],
                              LWristYaw_angle       = angles[9],
                              RWristYaw_angle       = angles[10],
                              LKneePitch_angle      = angles[11],
                              RKneePitch_angle      = angles[12]
                              )
        #self.set_pose_joint_space(self.motionProxy, None, angles)

    # convert poses with angles for adressed joints
    def convert_poses(self, poses, joint_name):
        points = []
        
        if joint_name == "rightElbow":
            for keypoint in poses[0].keypoints:
                if keypoint.part == "rightShoulder" or keypoint.part == "rightElbow" or keypoint.part == "rightWrist":
                    x,y = keypoint.position.x, keypoint.position.y 
                    points.append((x,y))
        elif joint_name == "leftElbow":
            for keypoint in poses[0].keypoints:
                if keypoint.part == "leftShoulder" or keypoint.part == "leftElbow" or keypoint.part == "leftWrist":
                    x,y = keypoint.position.x, keypoint.position.y 
                    points.append((x,y))
        elif joint_name == "rightShoulder":
            for keypoint in poses[0].keypoints:
                if keypoint.part == "rightHip" or keypoint.part == "rightShoulder" or keypoint.part == "rightElbow":
                    x,y = keypoint.position.x, keypoint.position.y
                    points.append((x,y))
        elif joint_name == "leftShoulder":
            for keypoint in poses[0].keypoints:
                if keypoint.part == "leftHip" or keypoint.part == "leftShoulder" or keypoint.part == "leftElbow":
                    x,y = keypoint.position.x, keypoint.position.y
                    points.append((x,y))
        elif joint_name == "rightKnee":
            for keypoint in poses[0].keypoints:
                if keypoint.part == "rightHip" or keypoint.part == "rightKnee" or keypoint.part == "rightAnkle":
                    x,y = keypoint.position.x, keypoint.position.y
                    points.append((x,y))
        else:
            print("joint name unkown")

        return self.find_angle(points[0], points[1], points[2])

    # calc the requested joint angle pos wrt to the model
    def find_angle(self, p1,p2,p3):
        p12 = math.sqrt(math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2))
        p13 = math.sqrt(math.pow((p1[0] - p3[0]), 2) + math.pow((p1[1] - p3[1]), 2))
        p23 = math.sqrt(math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2))
        resultRadian = math.acos(((math.pow(p12, 2)) + (math.pow(p13, 2)) - (math.pow(p23, 2))) / (2 * p12 * p13))
        return resultRadian

    '''
    not used:
    def convert_poses_rot(self, poses, joint_name):
        if joint_name == "RightLowArm":
            for keypoint in poses[0].keypoints:
                if keypoint.part == "rightShoulder" or keypoint.part == "rightElbow" or keypoint.part == "rightWrist":
                    x,y = keypoint.position.x, keypoint.position.y
                    points.append((x,y))
        else:
            print("joint name unkown")

        return self.calc_rot_angle(points[0], points[2])

    def calc_rot_angle(self, p1,p2):
        p12 = math.sqrt(math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2))
        p13 = math.sqrt(math.pow((p1[0] - p3[0]), 2) + math.pow((p1[1] - p3[1]), 2))
        p23 = math.sqrt(math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2))

        return
    '''

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

    # set position for requested joints
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
                                RWristYaw_angle,
                                LKneePitch_angle,
                                RKneePitch_angle):

        joint_angles_to_set = JointAnglesWithSpeed()
        # shoulder joint parameters
        joint_angles_to_set.joint_names.append("LShoulderPitch") # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(Lshoulder_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.joint_names.append("LShoulderRoll")
        joint_angles_to_set.joint_angles.append(LShoulderRoll_angle)
        joint_angles_to_set.joint_names.append("RShoulderPitch")
        joint_angles_to_set.joint_angles.append(Rshoulder_angle)
        joint_angles_to_set.joint_names.append("RShoulderRoll")
        joint_angles_to_set.joint_angles.append(RShoulderRoll_angle)
        # head joint parameters
        joint_angles_to_set.joint_names.append("HeadPitch")
        joint_angles_to_set.joint_angles.append(Head_angle)
        # elbow joint parameters
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
        # knee joint parameters
        joint_angles_to_set.joint_names.append("LKneePitch")
        joint_angles_to_set.joint_angles.append(LKneePitch_angle)
        joint_angles_to_set.joint_names.append("RKneePitch")
        joint_angles_to_set.joint_angles.append(RKneePitch_angle)
        # set parameters for joint move
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.joint_pub.publish(joint_angles_to_set)

    # check the requested angle with limits of joints and adjust angles, if necessary
    def joint_clamp(self, angles): 
        #HeadYaw_clamp = ...

        #LShoulderPitch    
        angles[0]=  max(-2.0857,    min(angles[0],  2.0857))
        #LShoulderRoll    
        angles[1]=  max(-0.3142,    min(angles[1],  1.3265))
        #RShoulderPitch
        angles[2]=  max(-2.0857,    min(angles[2],  2.0857))
        #RShoulderRoll
        angles[3]=  max(-1.3265,    min(angles[3],  0.3142))
        #HeadPitch
        angles[4]=  max(-0.6720,    min(angles[4],  0.5149))
        #LElbowRoll
        angles[5]=  max(-1.5446,    min(angles[5],  -0.0349))
        #RElbowRoll
        angles[6]=  max(0.0349,     min(angles[6],  1.5446))
        #LElbowYaw
        angles[7]=  max(-2.0857,    min(angles[7],  2.0857))
        #RElbowYaw
        angles[8]=  max(-2.0857,    min(angles[8],  2.0857))
        #LWristYaw
        angles[9]=  max(-1.8238,    min(angles[9],  1.8238))
        #RWristYaw
        angles[10]= max(-1.8238,    min(angles[9],  1.8238))
        #LKneePitch
        angles[11]= max(-0.092346,  min(angles[9],  2.112528))
        #RKneePitch
        angles[12]= max(-0.103083,  min(angles[9],  2.120198)) 

        return angles

    def degree_to_rad_convertion(self, angles): # convert all degree values to radians
        angles = [i * almath.TO_RAD for i in angles]
        return angles

    '''
    depricated
    def set_pose_joint_space(self, motionProxy, step ,angles=None): #update joints with interpolation

        # set the names for interpolation movement
        names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw"] 
        names += ["LElbowRoll", "LWristYaw"] # "LHand" not included in movement, extra call
        names += ["RShoulderPitch", "RShoulderRoll", "RElbowYaw"]
        names += ["RElbowRoll", "RWristYaw"] # "RHand" not included in movement, extra call

        # set parameters for interpolation movement
        time_len = 2.0
        timeLists = []
        for x in names:
            timeLists  += [time_len] #interpolation with time
        isAbsolute = True

        if angles == None:
            if (step =="init"): # first cycle
                # angles = calc_angles() -> later fct for calling init angle values
                angles = [0, 70, 0, 0, 0, 0, -70, 0, 0, 0]
                # check the max. angle of joints
                angles = self.joint_clamp(angles)

                # convert degrees in radian
                angles = self.degree_to_rad_convertion(angles)

                # command to change joints in group; non-blocking fct for interpolation
                self.motionProxy.post.angleInterpolation(names, angles, timeLists, isAbsolute) 
                time.sleep(0.5)
            elif (step=="update"): # following cycles 

                # Call getTaskList to have the previous angleInterpolation task number
                taskList = self.motionProxy.getTaskList()
                
                # Prepare the next target group (follow cycle) -> ranking wrt "names"!
                angles = [10, -10, 10, -10, 10, 10, 10, 10, 10, 10]
                # angles = calc_angles() -> later fct for calling update angle values 

                # check the max. angle of joints
                angles = self.joint_clamp(angles)
                
                # convert degrees in radian
                angles = self.degree_to_rad_convertion(angles)

                # command to change joints in group; 
                self.motionProxy.post.angleInterpolation(names, angles, timeLists, isAbsolute)
                time.sleep(0.5)

                # Kill the first angleInterpolation
                # smoothly from the current joint position and velocity
                self.motionProxy.killTask(taskList[0][1])
            else:
                print "Error occured in setting joint target"
                print "-------------------------------------"
                exit(1)
        else:
            # check the max. angle of joints
            angles = self.joint_clamp(angles)

            # convert degrees in radian
            # angles = self.degree_to_rad_convertion(angles)

            # command to change joints in group; non-blocking fct for interpolation
            self.motionProxy.post.angleInterpolation(names, angles, timeLists, isAbsolute) 
            time.sleep(0.5)
    '''

    '''
    not used:
    def central_control(self):
        # create several topic subscriber #
        rospy.Subscriber("key", String, self.keyboard_data) # for keyboard
        rospy.Subscriber("joint_states", JointState, self.joints_data) # for joint states
        self.jointPub = rospy.Publisher("joint_angles", JointAnglesWithSpeed, queue_size=10)

        # set parameters #
        obtain_pos = 0
        progress = 0

        # call service #
        #s = rospy.Service("move_joints_service", MoveJoints, Control.handle_move_joints)

        ## set stiffness ON: ##
        # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        self.set_stiffness(True)

        # init_posture, Zero (NOT "T-position")
        postureProxy = ALProxy("ALRobotPosture", self.robotIP, self.PORT)
        postureProxy.goToPosture("StandZero", 0.6)
        rospy.sleep(0.5)
        self.set_joint_angles(0,1.1,0,-1.1,0,0,0,0,0,0,0)
        rospy.sleep(0.5)


        # while True:
        #     if self.key == 'h': # set the selected joints in 'Zero position'
        #         #self.set_joint_angles(0,0,0,0,0,0,0,0,0,0,0,0)
        #         postureProxy.goToPosture("StandZero", 0.5)
        #     elif self.key == 'i': # set the init 'T-position'
        #         self.set_joint_angles(0,1.1,0,-1.1,0,0,0,0,0,0,0)
        #         rospy.sleep(1.0)
        #     elif self.key == 'g': # get position
        #         #DEPRECATED; not used for this project anymore
        #         # cartesian: get position
        #         if obtain_pos == 0:
        #             print("requested joint: {}".format(name))
        #             current = self.motionProxy.getPosition(name, frame, useSensorValues)
        #             print("Position (meters) of {} in Torso is:\nx = {},\ny = {},\nz = {}"
        #                                                         .format(name,
        #                                                                 current[0],
        #                                                                 current[1],
        #                                                                 current[2]))
        #             print("Orientation (radians) of {} in Torso is:\nroll  = {},\npitch = {},\nyaw   = {}"
        #                                                         .format(name,
        #                                                                 current[3], 
        #                                                                 current[4], 
        #                                                                 current[5]))
        #             print("=====================================================")
        #             obtain_pos = 1
                    
        #     elif self.key == 'r': 
        #         #DEPRECATED; not used for this project anymore
        #         # cartesian: set position interpolation in relative values
        #         if obtain_pos == 1:
        #             dx = targetPos[0] + current[0]
        #             dy = targetPos[1] + current[1]
        #             dz = targetPos[2] + current[2]
        #             droll  = targetPos[3]  + current[3]
        #             dpitch = targetPos[4]  + current[4]
        #             dyaw   = targetPos[5]   + current[5]

        #             targetPos = [dx, dy, dz, droll, dpitch, dyaw]
        #             self.motionProxy.positionInterpolation(effector, space, path,
        #                                     axisMask, times, isAbsolute)
        #             print("motion completed")
        #             obtain_pos = 0

        #     elif self.key == 'a':
        #         #DEPRECATED; not used for this project anymore
        #         # cartesian: set position interpolation in absolute values
        #         if obtain_pos == 1:
                    
        #             self.motionProxy.positionInterpolation(effector, space, path,
        #                                     axisMask, times, isAbsolute)
        #             print("motion completed")
        #             obtain_pos = 0

        #     elif self.key == 'f': #refresh setting para
        #         obtain_pos = 0
        #         progress = 0

        #     elif self.key == 't': 
        #         # times interpolations; here: 'Head' (split in 'HeadYaw' & 'HeadPitch')
        #         names      = ["HeadYaw", "HeadPitch"]
        #         angleLists = [[1.0, -1.0, 1.0, -1.0], [-1.0]]
        #         times      = [[1.0,  2.0, 3.0,  4.0], [ 5.0]]
        #         isAbsolute = False
        #         self.motionProxy.angleInterpolation(names, angleLists, times, isAbsolute)

        #     elif self.key == 'c': 
        #         # reactive control without interpolation; 

        #         #here: 'LArm & RArm together'
        #         names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw"] 
        #         names += ["LElbowRoll", "LWristYaw", "LHand"]
        #         names += ["RShoulderPitch", "RShoulderRoll", "RElbowYaw"]
        #         names += ["RElbowRoll", "RWristYaw", "RHand"]
        #         fractionMaxSpeed = 0.15

        #         # target
        #         angles = [10*almath.TO_RAD, -10*almath.TO_RAD, 10*almath.TO_RAD]
        #         angles += [-10*almath.TO_RAD, 10*almath.TO_RAD, 1]
        #         angles += [10*almath.TO_RAD, 10*almath.TO_RAD, 10*almath.TO_RAD]
        #         angles += [10*almath.TO_RAD, 10*almath.TO_RAD, 1]
        #         self.motionProxy.setAngles(names,angles,fractionMaxSpeed)
        #         # wait half a second
        #         #time.sleep(0.5)
        #         time.sleep(3.0)

        #         # change target
        #         angles = [0*almath.TO_RAD, 70*almath.TO_RAD, 0*almath.TO_RAD]
        #         angles += [0*almath.TO_RAD, 0*almath.TO_RAD, 0]
        #         angles += [0*almath.TO_RAD, -70*almath.TO_RAD, 0*almath.TO_RAD]
        #         angles += [0*almath.TO_RAD, 0*almath.TO_RAD, 0]
        #         self.motionProxy.setAngles(names,angles,fractionMaxSpeed)
        #         # wait half a second
        #         #time.sleep(0.5)
        #         time.sleep(3.0)                

        #         # change target
        #         # ...


        #     elif self.key == 'v': 
        #         # reactive control with interpolation; 
        #         # if obtain_pos == 1:
        #             #while True: -> or also establish a key press as abortion like 'while not self.key =..'
        #             if progress == 0:
        #                 step = "init"
        #                 self.set_pose_joint_space(self.motionProxy,step)
        #                 progress += 1
        #             elif progress == 1:
        #                 step = "update"
        #                 self.set_pose_joint_space(self.motionProxy,step)
        #             else:
        #                 print "Error occured in setting joint target"
        #                 print "-------------------------------------"
        #                 exit(1)

        #             # here: end of while loop

        #             #time.sleep(2.0)

        #             # obtain_pos += 0


        #     elif self.key == 'l': 
        #         # Example showing how to get the current task list
        #         # We will create a task first, so that we see a result
        #         names      = "HeadYaw"
        #         angleLists = 1.0
        #         timeList   = 3.0
        #         isAbsolute = True
        #         self.motionProxy.post.angleInterpolation(names, angleLists, timeList, isAbsolute)
        #         time.sleep(0.1)
        #         print 'Tasklist: ', self.motionProxy.getTaskList()

        #         time.sleep(2.0)

        #     elif self.key == 'k': 
        #         # This function is useful to kill motion Task
        #         # To see the current motionTask please use getTaskList() function

        #         self.motionProxy.post.angleInterpolation('HeadYaw', 90*almath.TO_RAD, 10, True) # (names, angleLists, timeList, isAbsolute)
        #         time.sleep(3)
        #         taskList = self.motionProxy.getTaskList()
        #         uiMotion = taskList[0][1]
        #         self.motionProxy.killTask(uiMotion)

        #         self.motionProxy.setStiffnesses("Head", 0.0)

        #         # alternative 1): kill move
        #         # self.motionProxy.post.moveTo(0.5, 0.0, 0.0)
        #         # time.sleep(3.0)
        #         # # End the walk suddenly (around 20ms)
        #         # self.motionProxy.killMove()

        #         # alternative 2): kill ALL tasks
        #         # # Example showing how to kill all the tasks.
        #         # self.motionProxy.killAll()
        #         # print "All tasks killed."

        #     elif self.key == 'o': 
        #         # Example showing how to open the left hand
        #         self.motionProxy.openHand('LHand')
        #         # Example showing how to close the right hand.
        #         handName  = 'LHand'
        #         self.motionProxy.closeHand(handName)

        #     elif self.key == 'b': 
        #         # Example that finds the difference between the command and sensed angles.
        #         names         = "Body"
        #         useSensors    = False
        #         commandAngles = self.motionProxy.getAngles(names, useSensors)
        #         print ("Command angles in 'Head' for 'Body': HeadYaw {}, HeadPitch {}"
        #                                                             .format(commandAngles[0],
        #                                                                     commandAngles[1]))          
        #         print ""                
        #         print ("Command angles in 'LArm' for 'Body': LShoulderPitch {}, LShoulderRoll {}, LElbowYaw {}, LElbowRoll {}, LWristYaw {}, LHand{}"
        #                                                             .format(commandAngles[2],
        #                                                                     commandAngles[3],
        #                                                                     commandAngles[4],
        #                                                                     commandAngles[5],
        #                                                                     commandAngles[6],
        #                                                                     commandAngles[7])) 
        #         print ""                
        #         print ("Command angles in 'LLeg' for 'Body': LHipYawPitch {}, LHipRoll {}, LHipPitch {}, LKneePitch {}, LAnklePitch {}, RAnkleRoll{}"
        #                                                             .format(commandAngles[8],
        #                                                                     commandAngles[9],
        #                                                                     commandAngles[10],
        #                                                                     commandAngles[11],
        #                                                                     commandAngles[12],
        #                                                                     commandAngles[13])) 
        #         print ""
        #         print ("Command angles in 'RLeg' for 'Body': RHipYawPitch {}, RHipRoll {}, RHipPitch {}, RKneePitch {}, RAnklePitch {}, LAnkleRoll{}"
        #                                                             .format(commandAngles[14],
        #                                                                     commandAngles[15],
        #                                                                     commandAngles[16],
        #                                                                     commandAngles[17],
        #                                                                     commandAngles[18],
        #                                                                     commandAngles[19])) 
        #         print ""
        #         print ("Command angles in 'RArm' for 'Body': RShoulderPitch {}, RShoulderRoll {}, RElbowYaw {}, RElbowRoll {}, RWristYaw {}, RHand{}"
        #                                                             .format(commandAngles[20],
        #                                                                     commandAngles[21],
        #                                                                     commandAngles[22],
        #                                                                     commandAngles[23],
        #                                                                     commandAngles[24],
        #                                                                     commandAngles[25])) 
        #         print("=====================================================")
        #         print ""

        #         useSensors  = True
        #         sensorAngles = self.motionProxy.getAngles(names, useSensors)
        #         print ("Sensor angles in 'Head' for 'Body': HeadYaw {}, HeadPitch {}"
        #                                                             .format(sensorAngles[0],
        #                                                                     sensorAngles[1]))          
        #         print ""                
        #         print ("Sensor angles in 'LArm' for 'Body': LShoulderPitch {}, LShoulderRoll {}, LElbowYaw {}, LElbowRoll {}, LWristYaw {}, LHand{}"
        #                                                             .format(sensorAngles[2],
        #                                                                     sensorAngles[3],
        #                                                                     sensorAngles[4],
        #                                                                     sensorAngles[5],
        #                                                                     sensorAngles[6],
        #                                                                     sensorAngles[7])) 
        #         print ""                
        #         print ("Sensor angles in 'LLeg' for 'Body': LHipYawPitch {}, LHipRoll {}, LHipPitch {}, LKneePitch {}, LAnklePitch {}, RAnkleRoll{}"
        #                                                             .format(sensorAngles[8],
        #                                                                     sensorAngles[9],
        #                                                                     sensorAngles[10],
        #                                                                     sensorAngles[11],
        #                                                                     sensorAngles[12],
        #                                                                     sensorAngles[13])) 
        #         print ""
        #         print ("Sensor angles in 'RLeg' for 'Body': RHipYawPitch {}, RHipRoll {}, RHipPitch {}, RKneePitch {}, RAnklePitch {}, LAnkleRoll{}"
        #                                                             .format(sensorAngles[14],
        #                                                                     sensorAngles[15],
        #                                                                     sensorAngles[16],
        #                                                                     sensorAngles[17],
        #                                                                     sensorAngles[18],
        #                                                                     sensorAngles[19])) 
        #         print ""
        #         print ("Sensor angles in 'RArm' for 'Body': RShoulderPitch {}, RShoulderRoll {}, RElbowYaw {}, RElbowRoll {}, RWristYaw {}, RHand{}"
        #                                                             .format(sensorAngles[20],
        #                                                                     sensorAngles[21],
        #                                                                     sensorAngles[22],
        #                                                                     sensorAngles[23],
        #                                                                     sensorAngles[24],
        #                                                                     sensorAngles[25])) 
        #         print("=====================================================")
        #         print ""

        #         errors = []
        #         for i in range(0, len(commandAngles)):
        #             errors.append(commandAngles[i]-sensorAngles[i])
        #         print "Errors"
        #         print ("Errors in 'Head' for 'Body': HeadYaw {}, HeadPitch {}"
        #                                                             .format(errors[0],
        #                                                                     errors[1]))          
        #         print ""                
        #         print ("Errors in 'LArm' for 'Body': LShoulderPitch {}, LShoulderRoll {}, LElbowYaw {}, LElbowRoll {}, LWristYaw {}, LHand{}"
        #                                                             .format(errors[2],
        #                                                                     errors[3],
        #                                                                     errors[4],
        #                                                                     errors[5],
        #                                                                     errors[6],
        #                                                                     errors[7])) 
        #         print ""                
        #         print ("Errors in 'LLeg' for 'Body': LHipYawPitch {}, LHipRoll {}, LHipPitch {}, LKneePitch {}, LAnklePitch {}, RAnkleRoll{}"
        #                                                             .format(errors[8],
        #                                                                     errors[9],
        #                                                                     errors[10],
        #                                                                     errors[11],
        #                                                                     errors[12],
        #                                                                     errors[13])) 
        #         print ""
        #         print ("Errors in 'RLeg' for 'Body': RHipYawPitch {}, RHipRoll {}, RHipPitch {}, RKneePitch {}, RAnklePitch {}, LAnkleRoll{}"
        #                                                             .format(errors[14],
        #                                                                     errors[15],
        #                                                                     errors[16],
        #                                                                     errors[17],
        #                                                                     errors[18],
        #                                                                     errors[19])) 
        #         print ""
        #         print ("Errors in 'RArm' for 'Body': RShoulderPitch {}, RShoulderRoll {}, RElbowYaw {}, RElbowRoll {}, RWristYaw {}, RHand{}"
        #                                                             .format(errors[20],
        #                                                                     errors[21],
        #                                                                     errors[22],
        #                                                                     errors[23],
        #                                                                     errors[24],
        #                                                                     errors[25])) 
        #         print("=====================================================")
        #         print ""

        #     elif self.key == 'q':
        #         break            

        # ## set stiffness OFF: (not needed, when using self.motionProxy.rest ##
        # # always check that your robot is in a stable position before disabling the stiffness!!
        # # self.set_stiffness(False) 

        # rate = rospy.Rate(10) # sets the sleep time to 10ms                    

        # #rospy.sleep(3.0)
        # print("motion task ended")

        # # Set NAO to resting position
        # self.motionProxy.rest()
        # print("NAO resting")
        # time.sleep(2.0)
        
        # #while not rospy.is_shutdown():
        # #    self.set_stiffness(self.stiffness)
        # #    rate.sleep()
        
        # sys.exit(0)
        '''



if __name__ == '__main__':
    try:
        central_instance = Control()
        #central_instance.central_control()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass