#!/usr/bin/env python2.7
import rospy
import sys
from std_msgs.msg import String

def myPublishFunction():
    my_firstPublisher = rospy.Publisher('key', String, queue_size=1000)
    rospy.init_node('keyboard_publisher', anonymous=True)
    rate = rospy.Rate(10)
      
    while not rospy.is_shutdown():
        input_str = raw_input("Message to send: ")
        my_firstPublisher.publish(input_str)
        rate.sleep()


def main():
    myPublishFunction()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass