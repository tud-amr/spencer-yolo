#!/usr/bin/env python

#realsense_spencer_adaptor.py
# node for connecting realsence d435i output to spencer tracking package
# This code was used from https://gist.github.com/tim-fan/a2abe1fe15cce8111d06c5c2187c7e97

import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image

def convertImageEncoding(imageMsg, bridge):
    """
    convert given image from 16UC1 in mm to 32FC1 m
    using given cv bridge
    Refer https://github.com/spencer-project/spencer_people_tracking/issues/4
    """
    #convert from 16UC1 in mm to 32FC1 m
    cvImg = bridge.imgmsg_to_cv2(imageMsg)
    cvImg32F = cvImg.astype('float32') / 1000.0
    convertedImageMsg = bridge.cv2_to_imgmsg(cvImg32F)
    convertedImageMsg.header = imageMsg.header
    return convertedImageMsg

def runAdaptor():
    rospy.init_node('realsense_spencer_adaptor')
    bridge = cv_bridge.CvBridge()

    pub = rospy.Publisher('converter/rgbd_front_top/depth/image_rect', Image, queue_size=10)
    callback = lambda imgMsg : pub.publish(convertImageEncoding(imgMsg, bridge))
    sub = rospy.Subscriber('/spencer/sensors/rgbd_front_top/depth/image_rect_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    runAdaptor()
