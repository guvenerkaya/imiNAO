#!/usr/bin/env python2.7
# Team C: Yassine El Himer, Gueven Erkaya, Patrick Hinz

import rospy 
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
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
        self.depth_pub = rospy.Publisher('/perceptor/depth', String, queue_size=10)

    def image_cb(self, data):
        self.img = np.zeros((240,320,3))

        bridge_instance = CvBridge()
        try:
            img = bridge_instance.imgmsg_to_cv2(data, "bgr8")
            ball_area = self.detect_based_on_color(img)
            self.depth_pub.publish(str(ball_area))

        except CvBridgeError as e:
            rospy.logerr(e) 

    def poses_cb(self, poses_msg):
        self.draw(poses_msg.poses)


    def detect_based_on_color(self, cv_image):
        # Red detection -- 2 masks required
        cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_range1 = np.array([0,140,0])
        upper_range1 = np.array([15,255,255])
        mask1 = cv2.inRange(hsv, lower_range1, upper_range1)

        lower_range2 = np.array([170,140,0])
        upper_range2 = np.array([185,255,255])
        mask2 = cv2.inRange(hsv, lower_range2, upper_range2)

        mask = mask1 + mask2

        # Convert mask to binary image
        ret,thresh = cv2.threshold(mask,127,255,0)

        # Find countours in the binary image
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]

        biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
        
        #Debug display
        cv2.imshow("object mask", mask)
        cv2.waitKey(3)
        return cv2.contourArea(biggest_contour)

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
