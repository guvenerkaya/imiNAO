#!/usr/bin/env python2.7
# Team C: Yassine El Himer, Gueven Erkaya, Patrick Hinz

import rospy 
import numpy as np
from sensor_msgs.msg import Image
from perceptor.msg import Poses
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

    

    # draw() will not show anything until poses are found
    def draw(poses):
        if len(poses) > 0:
            image(img, 0, 0, width, height)
            drawSkeleton(poses)
            drawKeypoints(poses)
            noLoop() # stop looping when the poses are estimated
    
    # A function to draw ellipses over the detected keypoints
    def drawKeypoints(self, poses):
        # Loop through all the poses detected
        for pose in poses:
            # For each pose detected, loop through all the keypoints
            for keypoint in pose.keypoints:
                # A keypoint is an object describing a body part (like rightArm or leftShoulder)
                # Only draw an ellipse is the pose probability is bigger than 0.2
                if keypoint.score > 0.2:
                    fill(255)
                    stroke(20)
                    strokeWeight(4)
                    ellipse(round(keypoint.position.x), round(keypoint.position.y), 8, 8)

    # A function to draw the skeletons
    def drawSkeleton(self, poses):
        img = np.zeros((240,320))
        #Loop through all the skeletons detected
        for pose in poses:
            skeleton = pose.skeleton
            #For every skeleton, loop through all body connections
            for j in range(0,len(skeleton)):
                partA = skeleton[j][0]
                partB = skeleton[j][1]
                cv2.line(img,(partA.position.x, partA.position.y),(partB.position.x, partB.position.y),(255,255,255),5)
        cv2.imshow("keypoints", img)

    
    def poses_cb(self, data):
        self.drawSkeleton(data)

    def run(self):
        rospy.init_node("my_subscriberNode", anonymous=True)
        rospy.Subscriber("/nao_robot/camera/"+self.camera+"/camera/image_raw", Image, self.image_cb)
        rospy.spin()


    def run2(self):
        rospy.init_node("my_subscriberNode", anonymous=True)
        rospy.Subscriber("/perceptor/poses", Poses, self.poses_cb)
        rospy.spin()


if __name__ == '__main__':
    try:
        perceptor = Perceptor("top")
        perceptor.run2()
    except rospy.ROSInterruptException:
        pass
