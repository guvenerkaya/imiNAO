#!/usr/bin/env python2.7
import rospy
import sys
from naoqi import ALProxy
from control_v2.srv import GetJoint,GetJointResponse

motionProxy = 0

# service handler
def handle_joint_info(request):
    print("requested joint: {}".format(request.name))
    current = motionProxy.getPosition(request.name, request.frame, request.useSensorValues)
    print("Position (meters) of {} in Torso is:\nx = {},\ny = {},\nz = {}"
                                                       .format(request.name,
                                                               current[0],
                                                               current[1],
                                                               current[2]))
    print("Orientation (radians) of {} in Torso is:\nroll  = {},\npitch = {},\nyaw   = {}"
                                                    .format(request.name,
                                                            current[3], 
                                                            current[4], 
                                                            current[5]))
    print("=====================================================")
    return GetJointResponse(current)

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print(sys.argv[2])
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('get_joint_server')
    s = rospy.Service("get_joint_service", GetJoint, handle_joint_info)
    rospy.spin()	
