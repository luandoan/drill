#!/usr/bin/env python

""" circle_detect using Depth Sense camera
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
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# function calculate distance from 2 points in xOy frame
def distance(x1,y1,x2,y2):
    xd = x1 - x2
    yd = y1 - y2
    return math.sqrt(xd*xd + yd*yd)


class circle_detect:
    def __init__(self):
        self.image_pub = rospy.Publisher("circle_detection", Image, queue_size = 10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rgb_image", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Actual content code
        #  code fiding circles and publish the closest circle

        # initialize key variables
        self.feature_point = list()
        self.tracked_point = list()
        self.mask = None

    #def process_image(self, cv_image):
    #    try:
        # Mask image
        self.mask = np.zero_like(cv_image)
        x, y, w, h = self.frame_width/4, self.frame_height/4, self.frame_width/2, frame_height/2
        self.mask[y:y+h, x:x+h]
        masked_image = cv2.bitwise_and(cv_image, self.mask)

        # create grey scale version of the image
        grey = cv2.cvtColor(masked_image, cv2.COLOR_RGB2GRAY)

        # Equalize the histogram to reduce lighting effect
        grey = cv2.equalizeHist(grey)

        # Remote salt-and-pepper, and white noise by using a combination of a median and Gaussian filter
        grey = cv2.medianBlur(grey, 5)
        grey = cv2.GaussianBlur(grey, 5)

        # Get the HoughCircles feature closest to the image center
        feature_point = self.get_feature_point(grey)

        if feature_point is not None and len(feature_point) > 0:
            # draw the center of the circles
            cv2.circle(self.marker_image, (feature_point[0], feature_point[1]), self.feature_size, (0, 0, 255, 0), cv.CV_FILLED, 8, 0)
            # draw the outer circle
            cv2.circle(self.marker_image, (feature_point[0], feature_point[1]), feature_point[2], (0, 255, 0, 0), self.feature_size, 8, 0)

            # convert feature_point from image coordinates to world coordinates
            feature_point = np.dot(np.linalg.pinv(self.projectionMatrix), np.array([feature_point[0], feature_point[1], 1]))
            feature_point = feature_point / feature_point[2]
            feature_point = np.array((feature_point[0], feature_point[1]))

            # provide self.tracked point to publish_poi to be published on the /poi topic
            self.tracked_point = feature_point

        return cv_image

    def get_feature_point(self, input_image):
        # initialize key variables
        feature_points = list()
        feature_point = list()

        # compute the x, y and radius of variable circle feature
        fp = cv.HoughCircles(input_image, **self.HoughCircles_params)
        if fp is not None and len(fp) > 0:
            # convert fp to numpy array
            feature_points = np.float32(fp).reshape(-1,3)

            # compute the circle closest to the camera's center
            dists = distance(feature_point[:,0], feature_point[:,1], w, h)
            sorted_dist_indicies = np.argsort(dists)
            sorted_feature_points = feature_points[sorted_dist_indicies]
            feature_point = sorted_feature_points[0]

        return feature_point



        cv2.imshow("Image Windows", feature_point)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


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
