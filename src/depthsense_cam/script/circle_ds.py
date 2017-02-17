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
        x0 = 320
        y0 = 240
        idx = None
        minC = 800
        numC = None

        img = cv2.medianBlur(cv_image,5)
        imgg = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        cimg = cv2.cvtColor(imgg, cv2.COLOR_GRAY2BGR)
        circles = cv2.HoughCircles(imgg,cv2.cv.CV_HOUGH_GRADIENT, 1.2, 100,  param1 = 200, param2 = 30, minRadius = 5, maxRadius = 50)


        if circles is None:
           cv2.imshow("Image Windows", cv_image)
           rospy.loginfo('No circles detected')
        #circles = np.uint16(np.around(circles))  # .astype("int")     # convert the (x,y) coordinates and radius of the circles to integers
        #numC = len(circles[0,:])
        else:
            print circles.shape


        # rospy.loginfo('circles found, number of circles {}'.format(numC))

        # for i in circles:
        #     dist[i] = distance(x0,y0, i[0],i[1])
        #     if dist[i] < minC:
        #         minC = dist[i]
        #         idx = i

        #for i in circles[0,:]:
        # print idx
        # cv2.circle(cv_image, (idx[0],idx[1]),idx[2], (0,255,0),1)     # draw the outer cirlce
        # cv2.circle(cv_image, (idx[0],idx[1]), 2, (0,0,255), 3)      # draw the center of the circle

            for i in circles[0,:]:
                cv2.circle(cv_image, (i[0],i[1]),i[2], (0,255,0),1)     # draw the outer cirlce
                cv2.circle(cv_image, (i[0],i[1]), 2, (0,0,255), 3)      # draw the center of the circle

        cv2.imshow("Image Windows", cv_image)
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
