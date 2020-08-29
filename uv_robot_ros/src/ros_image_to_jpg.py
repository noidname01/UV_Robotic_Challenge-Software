#!/usr/bin/env python2.7
import rospy
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def imgProcess(image_message):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    cv2.imwrite('/home/noidname/UV_Robotic_Challenge-Software/uv-robotic-web/src/image/camera_stream.jpg', cv_image)
    

def listener():

    rospy.init_node('camera_stream_listener', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, imgProcess)

    rospy.spin()

if __name__ == "__main__":
    listener()


