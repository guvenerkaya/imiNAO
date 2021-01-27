#!/usr/bin/env python2.7
# Team C: Yassine El Himer, Gueven Erkaya, Patrick Hinz

import rospy 
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class Perceptor:
    def __init__(self, camera):
        self.camera = camera

        # camera matrix parameters for NAO camera; given through tutorial
        self.cameraMatrix = np.array([[551.543059,  0.000000  , 327.382898],
                                      [0.000000  ,  553.736023, 225.026380],
                                      [0.000000  ,  0.000000  , 1.000000]])
        
        # distortion parameters of NAO camera; given through tutorial
        self.distCoeffs = np.array([-0.066494,0.095481,-0.000279,0.002292,0.000000])

    def image_cb(self, data):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data, "bgr8")
            
            #Convert to grayscale image
            cv_gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            cv2.imshow(self.camera + " Camera Images", cv_image)
            cv2.waitKey(2)

        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        rospy.init_node("my_subscriberNode", anonymous=True)
        rospy.Subscriber("/nao_robot/camera/"+self.camera+"/camera/image_raw", Image, self.image_cb)
        rospy.spin()


if __name__ == '__main__':
    try:
        perceptor = Perceptor("top")
        perceptor.run()
    except rospy.ROSInterruptException:
        pass
