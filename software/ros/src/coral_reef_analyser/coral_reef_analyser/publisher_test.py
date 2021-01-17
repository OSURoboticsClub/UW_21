#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# !!ImageArr is a customly added msg, change this accordingly to what the publisher uses
from coral_reef_analyser.msg import ImageArr

def start_node():
    rospy.init_node("image_pub")
    rospy.loginfo("image_pub node has started")

    img1 = cv2.imread("test_img/past.png")
    img2 = cv2.imread("test_img/present.png")
    rospy.loginfo("Image loaded")
    bridge = CvBridge()
    imgMsg0 = bridge.cv2_to_imgmsg(img1, "bgr8")
    imgMsg1 = bridge.cv2_to_imgmsg(img2, "bgr8")
    imgMsg = ImageArr()
    imgMsg.img1 = imgMsg0
    imgMsg.img2 = imgMsg1
    pub = rospy.Publisher('image', ImageArr, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(imgMsg)
        rospy.Rate(25.0).sleep()  # 1 Hz

if __name__ == "__main__":
    start_node()
