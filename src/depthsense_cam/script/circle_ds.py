#!/usr/bin/env python

"""	circle_detect using Depth Sense camera
	based on HoughCircles python ROS
	Detect all possible circles and point out the interested one, which closest
	to center of image frame or any other interest points.
	Special applied to CCAM project and End-Effector design

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
from std_msgs.msg import String, MultiArrayLayout, MultiArrayDimension
from rospy_tutorials.msg import Floats
from cv_bridge import CvBridge, CvBridgeError


class circle_detect:
	def __init__(self):
		self.image_pub = rospy.Publisher("circle_detection", Image, queue_size = 10)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/rgb_image", Image, self.callback)
		self.depth_sub = rospy.Subscriber("/depth_image", Image, self.depth_img)
		self.point_pub = rospy.Publisher("point_support", Floats, queue_size = 5)

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		sz = np.shape(cv_image)

		# Actual content code
		#  code fiding circles and publish the closest circle
		x0 = 320
		y0 = 240
		idx = None
		minC = 800
		numC = None
		
		img = cv2.medianBlur(cv_image,5)
		imgg = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		
		cimg = cv2.cvtColor(imgg, cv2.COLOR_GRAY2BGR)
		circles = cv2.HoughCircles(imgg,cv2.cv.CV_HOUGH_GRADIENT, 1.2, 100,  param1 = 200, param2 = 30, minRadius = 5, maxRadius = 50)

		if circles is  not None:
			# get the number of circles detected
			sp = np.shape(circles)
			nc = sp[1]
			rospy.loginfo("Circle found: %d", nc)
			#print nc
		
			#find the closest circle regarding center point
			data = np.uint16(np.around(circles)).reshape(-1,3)
			dists = np.zeros(nc)
			mindist = 2000
			idx = 0
			for i in range(nc):
				dists[i] = np.sqrt((data[i,0]-sz[1]/2)**2 + (data[i,1]-sz[0]/2)**2)
				if (dists[i] < mindist):
					mindist = dists[i]
					idx = i
			best = data[idx]
			print "The closest circle: ", np.round(mindist), idx, best 
				
			# Draw selected circle and point out some information
			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(cv_image, 'Image size is: ' + str(sz[0]) + "x" + str(sz[1]),  (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0)) 
			cv2.putText(cv_image, 'Number of detected circles: ' + str(nc),  (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0)) 
			cv2.putText(cv_image, 'The selected wrench position is: ' + str(best[0]) + "x" + str(best[1]),  (10,90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0)) 
			
			# draw the center and outer circle
			cv2.circle(cv_image, (best[0],best[1]), 2, (0,0,255), 3)
			cv2.circle(cv_image, (best[0],best[1]), best[2], (0,255,0), 2)

			# pick 3 random points on detected circle
			cv2.circle(cv_image, (best[0],best[1]-best[2]), 2, (0,0,255), 3)
			cv2.circle(cv_image, (best[0]-best[2],best[1]), 2, (0,0,255), 3)
			cv2.circle(cv_image, (best[0]+best[2],best[1]), 2, (0,0,255), 3)

			# pick 3 random points closed to circle
			cv2.circle(cv_image, (best[0],best[1]-best[2]-10), 2, (255,0,0), 3)
			cv2.circle(cv_image, (best[0]-best[2]-10,best[1]), 2, (255,0,0), 3)
			cv2.circle(cv_image, (best[0]+best[2]+10,best[1]), 2, (255,0,0), 3)

			# pick 3 random points closed to circle
			cv2.circle(cv_image, (best[0],best[1]+best[2]+20), 2, (255,255,0), 3)
			cv2.circle(cv_image, (best[0]-best[2]-20,best[1]), 2, (255,255,0), 3)
			cv2.circle(cv_image, (best[0]+best[2]+20,best[1]), 2, (255,255,0), 3)

			# publish interested points to array
			point = np.array([ best[0],best[1]-best[2], best[0]-best[2],best[1], best[0]+best[2],best[1], best[0],best[1]-best[2]-10, best[0],best[1]+best[2]+10, best[0],best[1]+best[2]+20, best[0]-best[2]-20,best[1], best[0]+best[2]+20,best[1]])
			self.point_pub.publish(point)
				

		else:
			cv2.imshow("Image Windows", cv_image)
			rospy.loginfo('No circles detected')
			
		
		cv2.imshow("Image Windows", cv_image)
		cv2.waitKey(3)

		# process depth image
		#depth_data = self.depth_img(data)
		#cv_depth = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
		#cv2.imshow("Depth Image", cv_depth)		


		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
			self.point_pub.publish(point)
		except CvBridgeError as e:
			print(e)

	def depth_img(self,data):
		self.data = data
		return data

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
