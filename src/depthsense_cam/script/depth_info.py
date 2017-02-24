#!/usr/bin/env python

""" 	distance definition
	get distance from some interest points for getting orientation
	and adjusting values 
	Created by Luan Doan
	CMS Lab, VTech Feb 2017
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
		self.point_sub = rospy.Subscriber("/point_support", Floats, self.update_pos)
		self.depth_pub = rospy.Publisher("/depth_process", Image, queue_size = 1)

	def callback(self, data):
		try:
			# read depth image from ros topic
			cv_depth = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		if cv_depth is not None:
			# convert image to matrix
			depth_matrix = np.asarray(cv_depth)
			sz = np.shape(depth_matrix)
			
			#rospy.loginfo("Size of depth image: ,%s", sz)

			#distance = depth_matrix(160,240,3)
			dt = cv_depth[160,120]			
			print dt #
			dist = dt[0]
			print "Distance: ", dist
			



			# put text on depth image
			#cv2.putText(cv_depth, 'Distance from the center of image is: ', str(dist),  (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255,0,0))

			cv2.imshow("Depth Image", cv_depth)
			cv2.waitKey(3)

			try:
				self.depth_pub.publish(self.bridge.cv2_to_imgmsg(cv_depth, "bgr8"))
			except CvBridgeError as e:
				print (e)
		
		# depth = cv_ptr->image.at<short int>(cv::Point(240,320))
	def update_pos(self, pos):
		self.pos = pos

def main(args):
	dd = depth_distance()
	rospy.init_node('point_distance', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
