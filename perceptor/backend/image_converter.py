#!/usr/bin/env python2.7
import cv2
import rospy
import base64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self, camera):
        self.camera = camera    
        rospy.init_node("perceptor_node", anonymous=True)
        self.img_sub = rospy.Subscriber("/nao_robot/camera/"+self.camera+"/camera/image_raw", Image, self.image_cb)
        self.img_pub = rospy.Publisher('/perceptor/ImageConverted', String, queue_size=10)

    def image_cb(self, data):
        bridge_instance = CvBridge()
        try:
            img = bridge_instance.imgmsg_to_cv2(data, "bgr8")
            _, buffer = cv2.imencode('.jpg', img)
            image_as_str = base64.b64encode(buffer).decode('utf-8')
            self.img_pub.publish(image_as_str)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        converter = ImageConverter("top")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
