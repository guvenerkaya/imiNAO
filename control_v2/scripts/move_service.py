#!/usr/bin/env python2.7
import rospy
import time
import sys
import motion
import numpy as np
import cv2
import almath
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

    def joint_clamp():
        #HeadPitch_clamp = ...
        #HeadYaw_clamp = ...
        LShoulderPitch_clamp    = (-119.5*almath.TO_RAD ,119.5*almath.TO_RAD)
        LShoulderRoll           = (-18*almath.TO_RAD    ,76*almath.TO_RAD)
        LElbowYaw               = (-119.5*almath.TO_RAD ,119.5*almath.TO_RAD)
        LElbowRoll              = (-88.5*almath.TO_RAD  ,-2*almath.TO_RAD)
        LWristYaw               = (-104.5*almath.TO_RAD ,104.5*almath.TO_RAD)
        RShoulderPitch          = (-119.5*almath.TO_RAD ,119.5*almath.TO_RAD)
        RShoulderRoll           = (-76*almath.TO_RAD    ,18*almath.TO_RAD)
        RElbowYaw               = (-119.5*almath.TO_RAD ,119.5*almath.TO_RAD)
        RElbowRoll              = (2*almath.TO_RAD      ,88.5*almath.TO_RAD)
        RWristYaw               = (-104.5*almath.TO_RAD ,104.5*almath.TO_RAD)

    

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
        rospy.sleep(0.5)
        self.set_joint_angles(0,1.1,0,-1.1,0,0,0,0,0,0,0)
        rospy.sleep(0.5)

        
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
                '''DEPRECATED; not used for this project anymore
                # cartesian: get position
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
                    obtain_pos = 1'''
                    
            elif self.key == 'r': 
                '''DEPRECATED; not used for this project anymore
                # cartesian: set position interpolation in relative values
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
                    obtain_pos = 0'''

            elif self.key == 'a':
                '''DEPRECATED; not used for this project anymore
                # cartesian: set position interpolation in absolute values
                if obtain_pos == 1:
                    
                    motionProxy.positionInterpolation(effector, space, path,
                                            axisMask, times, isAbsolute)
                    print("motion completed")
                    obtain_pos = 0'''

            elif self.key == 'f': #refresh setting para
                obtain_pos = 0

            elif self.key == 't': 
                # times interpolations; here: 'Head' (split in 'HeadYaw' & 'HeadPitch')
                names      = ["HeadYaw", "HeadPitch"]
                angleLists = [[1.0, -1.0, 1.0, -1.0], [-1.0]]
                times      = [[1.0,  2.0, 3.0,  4.0], [ 5.0]]
                isAbsolute = False
                motionProxy.angleInterpolation(names, angleLists, times, isAbsolute)

            elif self.key == 'c': 
                # reactive control without interpolation; 

                #here: 'LArm & RArm together'
                names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw"] 
                names += ["LElbowRoll", "LWristYaw", "LHand"]
                names += ["RShoulderPitch", "RShoulderRoll", "RElbowYaw"]
                names += ["RElbowRoll", "RWristYaw", "RHand"]
                fractionMaxSpeed = 0.15

                # target
                angles = [10*almath.TO_RAD, -10*almath.TO_RAD, 10*almath.TO_RAD]
                angles += [-10*almath.TO_RAD, 10*almath.TO_RAD, 1]
                angles += [10*almath.TO_RAD, 10*almath.TO_RAD, 10*almath.TO_RAD]
                angles += [10*almath.TO_RAD, 10*almath.TO_RAD, 1]
                motionProxy.setAngles(names,angles,fractionMaxSpeed)
                # wait half a second
                #time.sleep(0.5)
                time.sleep(3.0)

                # change target
                angles = [0*almath.TO_RAD, 70*almath.TO_RAD, 0*almath.TO_RAD]
                angles += [0*almath.TO_RAD, 0*almath.TO_RAD, 0]
                angles += [0*almath.TO_RAD, -70*almath.TO_RAD, 0*almath.TO_RAD]
                angles += [0*almath.TO_RAD, 0*almath.TO_RAD, 0]
                motionProxy.setAngles(names,angles,fractionMaxSpeed)
                # wait half a second
                #time.sleep(0.5)
                time.sleep(3.0)                

                # change target
                # ...


            elif self.key == 'v': 
                # reactive control with interpolation; 
                
                # here: 'HeadYaw'

                # Head Start to zeros
                names             = "Head"
                targetAngles      = [0.0, 0.0]
                maxSpeedFraction  = 0.2 # Using 20% of maximum joint speed
                motionProxy.angleInterpolationWithSpeed(names, targetAngles, maxSpeedFraction)

                # Example showing a reactive control with time function angleInterpolation
                # Goal: after 1.0 second, retarget from 40 to 0 degree: smooth transition

                # Interpolate the head yaw to 20 degrees in 2.0 seconds
                # With post, angleInterpolation become non-blocking
                names      = "HeadYaw, HeadPitch"
                angleLists = [40.0*almath.TO_RAD, 10*almath.TO_RAD]
                timeLists  = [2.0, 2.0]
                isAbsolute = True
                motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)
                time.sleep(0.5)

                # Call getTaskList to have the previous angleInterpolation task number
                taskList = motionProxy.getTaskList()

                # Prepare the next target to 60.0 degrees in 1.0 second
                angleLists = [60.0*almath.TO_RAD, -10*almath.TO_RAD]
                timeLists  = [1.0, 1.0]
                motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)
                time.sleep(0.5)

                # Kill the first angleInterpolation (go to 40.0 degrees), the second start
                # smoothly from the current joint position and velocity (go to 0 degrees)
                motionProxy.killTask(taskList[0][1])

                time.sleep(2.0)

            elif self.key == 'l': 
                # Example showing how to get the current task list
                # We will create a task first, so that we see a result
                names      = "HeadYaw"
                angleLists = 1.0
                timeList   = 3.0
                isAbsolute = True
                motionProxy.post.angleInterpolation(names, angleLists, timeList, isAbsolute)
                time.sleep(0.1)
                print 'Tasklist: ', motionProxy.getTaskList()

                time.sleep(2.0)

            elif self.key == 'k': 
                # This function is useful to kill motion Task
                # To see the current motionTask please use getTaskList() function

                motionProxy.post.angleInterpolation('HeadYaw', 90*almath.TO_RAD, 10, True) # (names, angleLists, timeList, isAbsolute)
                time.sleep(3)
                taskList = motionProxy.getTaskList()
                uiMotion = taskList[0][1]
                motionProxy.killTask(uiMotion)

                motionProxy.setStiffnesses("Head", 0.0)

                # alternative 1): kill move
                # motionProxy.post.moveTo(0.5, 0.0, 0.0)
                # time.sleep(3.0)
                # # End the walk suddenly (around 20ms)
                # motionProxy.killMove()

                # alternative 2): kill ALL tasks
                # # Example showing how to kill all the tasks.
                # motionProxy.killAll()
                # print "All tasks killed."

            elif self.key == 'o': 
                # Example showing how to open the left hand
                motionProxy.openHand('LHand')
                # Example showing how to close the right hand.
                handName  = 'LHand'
                motionProxy.closeHand(handName)

            elif self.key == 'b': 
                # Example that finds the difference between the command and sensed angles.
                names         = "Body"
                useSensors    = False
                commandAngles = motionProxy.getAngles(names, useSensors)
                print ("Command angles in 'Head' for 'Body': HeadYaw {}, HeadPitch {}"
                                                                    .format(commandAngles[0],
                                                                            commandAngles[1]))          
                print ""                
                print ("Command angles in 'LArm' for 'Body': LShoulderPitch {}, LShoulderRoll {}, LElbowYaw {}, LElbowRoll {}, LWristYaw {}, LHand{}"
                                                                    .format(commandAngles[2],
                                                                            commandAngles[3],
                                                                            commandAngles[4],
                                                                            commandAngles[5],
                                                                            commandAngles[6],
                                                                            commandAngles[7])) 
                print ""                
                print ("Command angles in 'LLeg' for 'Body': LHipYawPitch {}, LHipRoll {}, LHipPitch {}, LKneePitch {}, LAnklePitch {}, RAnkleRoll{}"
                                                                    .format(commandAngles[8],
                                                                            commandAngles[9],
                                                                            commandAngles[10],
                                                                            commandAngles[11],
                                                                            commandAngles[12],
                                                                            commandAngles[13])) 
                print ""
                print ("Command angles in 'RLeg' for 'Body': RHipYawPitch {}, RHipRoll {}, RHipPitch {}, RKneePitch {}, RAnklePitch {}, LAnkleRoll{}"
                                                                    .format(commandAngles[14],
                                                                            commandAngles[15],
                                                                            commandAngles[16],
                                                                            commandAngles[17],
                                                                            commandAngles[18],
                                                                            commandAngles[19])) 
                print ""
                print ("Command angles in 'RArm' for 'Body': RShoulderPitch {}, RShoulderRoll {}, RElbowYaw {}, RElbowRoll {}, RWristYaw {}, RHand{}"
                                                                    .format(commandAngles[20],
                                                                            commandAngles[21],
                                                                            commandAngles[22],
                                                                            commandAngles[23],
                                                                            commandAngles[24],
                                                                            commandAngles[25])) 
                print("=====================================================")
                print ""

                useSensors  = True
                sensorAngles = motionProxy.getAngles(names, useSensors)
                print ("Sensor angles in 'Head' for 'Body': HeadYaw {}, HeadPitch {}"
                                                                    .format(sensorAngles[0],
                                                                            sensorAngles[1]))          
                print ""                
                print ("Sensor angles in 'LArm' for 'Body': LShoulderPitch {}, LShoulderRoll {}, LElbowYaw {}, LElbowRoll {}, LWristYaw {}, LHand{}"
                                                                    .format(sensorAngles[2],
                                                                            sensorAngles[3],
                                                                            sensorAngles[4],
                                                                            sensorAngles[5],
                                                                            sensorAngles[6],
                                                                            sensorAngles[7])) 
                print ""                
                print ("Sensor angles in 'LLeg' for 'Body': LHipYawPitch {}, LHipRoll {}, LHipPitch {}, LKneePitch {}, LAnklePitch {}, RAnkleRoll{}"
                                                                    .format(sensorAngles[8],
                                                                            sensorAngles[9],
                                                                            sensorAngles[10],
                                                                            sensorAngles[11],
                                                                            sensorAngles[12],
                                                                            sensorAngles[13])) 
                print ""
                print ("Sensor angles in 'RLeg' for 'Body': RHipYawPitch {}, RHipRoll {}, RHipPitch {}, RKneePitch {}, RAnklePitch {}, LAnkleRoll{}"
                                                                    .format(sensorAngles[14],
                                                                            sensorAngles[15],
                                                                            sensorAngles[16],
                                                                            sensorAngles[17],
                                                                            sensorAngles[18],
                                                                            sensorAngles[19])) 
                print ""
                print ("Sensor angles in 'RArm' for 'Body': RShoulderPitch {}, RShoulderRoll {}, RElbowYaw {}, RElbowRoll {}, RWristYaw {}, RHand{}"
                                                                    .format(sensorAngles[20],
                                                                            sensorAngles[21],
                                                                            sensorAngles[22],
                                                                            sensorAngles[23],
                                                                            sensorAngles[24],
                                                                            sensorAngles[25])) 
                print("=====================================================")
                print ""

                errors = []
                for i in range(0, len(commandAngles)):
                    errors.append(commandAngles[i]-sensorAngles[i])
                print "Errors"
                print ("Errors in 'Head' for 'Body': HeadYaw {}, HeadPitch {}"
                                                                    .format(errors[0],
                                                                            errors[1]))          
                print ""                
                print ("Errors in 'LArm' for 'Body': LShoulderPitch {}, LShoulderRoll {}, LElbowYaw {}, LElbowRoll {}, LWristYaw {}, LHand{}"
                                                                    .format(errors[2],
                                                                            errors[3],
                                                                            errors[4],
                                                                            errors[5],
                                                                            errors[6],
                                                                            errors[7])) 
                print ""                
                print ("Errors in 'LLeg' for 'Body': LHipYawPitch {}, LHipRoll {}, LHipPitch {}, LKneePitch {}, LAnklePitch {}, RAnkleRoll{}"
                                                                    .format(errors[8],
                                                                            errors[9],
                                                                            errors[10],
                                                                            errors[11],
                                                                            errors[12],
                                                                            errors[13])) 
                print ""
                print ("Errors in 'RLeg' for 'Body': RHipYawPitch {}, RHipRoll {}, RHipPitch {}, RKneePitch {}, RAnklePitch {}, LAnkleRoll{}"
                                                                    .format(errors[14],
                                                                            errors[15],
                                                                            errors[16],
                                                                            errors[17],
                                                                            errors[18],
                                                                            errors[19])) 
                print ""
                print ("Errors in 'RArm' for 'Body': RShoulderPitch {}, RShoulderRoll {}, RElbowYaw {}, RElbowRoll {}, RWristYaw {}, RHand{}"
                                                                    .format(errors[20],
                                                                            errors[21],
                                                                            errors[22],
                                                                            errors[23],
                                                                            errors[24],
                                                                            errors[25])) 
                print("=====================================================")
                print ""

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
        #rospy.spin()
        
        #while not rospy.is_shutdown():
        #    self.set_stiffness(self.stiffness)
        #    rate.sleep()
        
        sys.exit(0)


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