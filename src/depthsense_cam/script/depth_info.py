#!/usr/bin/env python

""" 	distance definition
	get distance from some interest points for getting orientation
	and adjusting values 
	Created by Luan Doan
	CMS Lab, VTech June 2016
"""

import rospy
import math
import roslib
import sys
import cv2
import time
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
class depth_distance:
	def __init__(self):
		self.distance_pub = rospy.Publisher("/point_distance", Floats, queue_size = 10)
		self.bridge = CvBridge()
		self.depth_sub = rospy.Subscriber("/depth_image", Image, self.callback)
		self.point_sub = rospy.Subscriber("/point_support", Float, self.update_pos)

	def callback(self, data):
		try:
			cv_depth = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		if cv_depth is not None:
			cv2.imshow("Depth Image", cv_depth)
			cv2.waitKey(3)

	def update_pos(self, pos):
		self.pos = pos

def main(args):
    ct = circle_detect()
    rospy.init_node('circle_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
