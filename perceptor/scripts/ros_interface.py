#!/usr/bin/env python2.7
# Team C: Yassine El Himer, Gueven Erkaya, Patrick Hinz

import rospy 
import numpy as np
from sensor_msgs.msg import Image
from perceptor.msg import Poses, Pose, Keypoint
from cv_bridge import CvBridge, CvBridgeError
import cv2


# information for skeleton
connected_part_names = [
['leftHip', 'leftShoulder'], ['leftElbow', 'leftShoulder'],
['leftElbow', 'leftWrist'], ['leftHip', 'leftKnee'],
['leftKnee', 'leftAnkle'], ['rightHip', 'rightShoulder'],
['rightElbow', 'rightShoulder'], ['rightElbow', 'rightWrist'],
['rightHip', 'rightKnee'], ['rightKnee', 'rightAnkle'],
['leftShoulder', 'rightShoulder'], ['leftHip', 'rightHip']]


class Perceptor:
    def __init__(self, camera):

        self.img = None

        self.camera = camera
        # camera matrix parameters for NAO camera; given through tutorial
        self.cameraMatrix = np.array([[551.543059,  0.000000  , 327.382898],
                                      [0.000000  ,  553.736023, 225.026380],
                                      [0.000000  ,  0.000000  , 1.000000]])
        
        # distortion parameters of NAO camera; given through tutorial
        self.distCoeffs = np.array([-0.066494,0.095481,-0.000279,0.002292,0.000000])

        rospy.init_node("perceptor_node", anonymous=True)
        self.img_sub = rospy.Subscriber("/nao_robot/camera/"+self.camera+"/camera/image_raw", Image, self.image_cb)
        self.pose_sub  = rospy.Subscriber("/perceptor/poses", Poses, self.poses_cb)

    def image_cb(self, data):
        # bridge_instance = CvBridge()
        # try:
        #     self.img = bridge_instance.imgmsg_to_cv2(data, "bgr8")
        # except CvBridgeError as e:
        #     rospy.logerr(e) 
        self.img = np.zeros((240,320,3))

    def poses_cb(self, poses_msg):
        self.draw(poses_msg.poses)


    # draw() will not show anything until poses are found
    def draw(self, poses):
        if len(poses) > 0:
            self.drawKeypoints(poses)
            self.drawSkeleton(poses)
        print(poses)
        cv2.imshow("keypoints", self.img)
        cv2.waitKey(3)
    
    # A function to draw ellipses over the detected keypoints
    def drawKeypoints(self, poses):
        # Loop through all the poses detected
        for pose in poses:
            self.keypoint_dict = {}
            # For each pose detected, loop through all the keypoints
            for keypoint in pose.keypoints:
                # A keypoint is an object describing a body part (like rightArm or leftShoulder)
                if keypoint.score > 0.8:
                    self.keypoint_dict[keypoint.part] = (int(keypoint.position.x), int(keypoint.position.y))
                    cv2.circle(self.img, (int(keypoint.position.x), int(keypoint.position.y)), 2, (255, 0, 0), 2)
   
        
    # A function to draw the skeletons
    def drawSkeleton(self, poses):
        # draw skeleton
        for connected_part1, connected_part2 in connected_part_names:
            if all(k in self.keypoint_dict for k in (connected_part1, connected_part2)):
                cv2.line(self.img, self.keypoint_dict[connected_part1], self.keypoint_dict[connected_part2], (0,0,255), 3)

if __name__ == '__main__':
    try:
        perceptor = Perceptor("top")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
