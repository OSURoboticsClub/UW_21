#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
# !!ImageArr is a customly added msg, change this accordingly to what the publisher uses
from coral_reef_analyser.msg import ImageArr
from cv_bridge import CvBridge
import cv2
from operations import *

# takes in img matrix and displays it 
def display(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

# Use functions in the operation.py to mark the differences between 
# the past and present condition of the coral reef
def process_img(msg):
    bridge = CvBridge()
    img1 = bridge.imgmsg_to_cv2(msg.img1, "bgr8")
    img2 = bridge.imgmsg_to_cv2(msg.img2, "bgr8")
    rospy.loginfo("Images received")

    warp_img1 = warpImage(img1, img2)
    rospy.loginfo("Images warped")

    img1_reclr, img2_reclr = filterColor(warp_img1), filterColor(img2)
    rospy.loginfo("Filtered Image Colors")

    res = getDiff_weak(img1_reclr, img2_reclr, img2)
    rospy.loginfo("Marked Image Differences")

    # TO DO
    # Should we save the img after processing? or simply display it?

def start_node():
    rospy.init_node('img_sub')
    rospy.loginfo('img_sub node has started')
    # !Again ImageArr is a customly written msg, change it accordingly to what the publisher uses
    rospy.Subscriber("image", ImageArr, process_img)
    rospy.spin()

if __name__ == '__main__':
    start_node()
