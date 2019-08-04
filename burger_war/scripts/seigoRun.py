#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt

# camera image 640*480
img_w = 640
img_h = 480

class SeigoBot():
    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # Lidar
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('/red_bot/scan', LaserScan, self.lidarCallback)
        self.front_distance = 10000 # init

        # usb camera
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/red_bot/image_raw', Image, self.imageCallback)
        self.red_angle = -1 # init
        self.blue_angle = -1 # init
        self.green_angle = -1 # init

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data

        # visualize scan data with radar chart
        plt.cla();
        angles = np.linspace(0, 2 * np.pi, len(self.scan.ranges) + 1, endpoint=True)
        values = np.concatenate((self.scan.ranges, [self.scan.ranges[0]]))
        ax = plt.subplot(111, polar=True)
        ax.plot(angles, values, 'o-')
        ax.fill(angles, values, alpha=0.25)
        ax.set_rlim(0, 2.5)
        plt.pause(0.05)
        # print(self.scan)
        # print(self.scan.ranges[0])
        self.front_distance = self.scan.ranges[0]

    def find_rect_of_target_color(self, image, color_type): # r:0, g:1, b:2
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]

        # red detection
        if color_type == 0:
            mask = np.zeros(h.shape, dtype=np.uint8)
            mask[((h < 20) | (h > 200)) & (s > 128)] = 255

        # blue detection
        if color_type == 2:
            lower_blue = np.array([130, 50, 50])
            upper_blue = np.array([200, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # green detection
        if color_type == 1:
            lower_green = np.array([75, 50, 50])
            upper_green = np.array([110, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

        # get contours
        img, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        return rects
        
    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # print(self.img);
        frame = self.img
        # red
        rects = self.find_rect_of_target_color(frame, 0)
        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
            # angle(rad)
            tmp_angle = ((rect[0:2]+rect[0:2]+rect[2:4])/2-(img_w/2)) *0.077
            self.red_angle = tmp_angle * np.pi / 180
            # print ( tmp_angle )

        # green
        rects = self.find_rect_of_target_color(frame, 1)
        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
            # angle(rad)
            tmp_angle = ((rect[0:2]+rect[0:2]+rect[2:4])/2-(img_w/2)) *0.077
            self.green_angle = tmp_angle * np.pi / 180
            # print ( tmp_angle )

        # blue
        rects = self.find_rect_of_target_color(frame, 2)
        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
            # angle(rad)
            tmp_angle = ((rect[0:2]+rect[0:2]+rect[2:4])/2-(img_w/2)) *0.077
            self.blue_angle = tmp_angle * np.pi / 180
            # print ( tmp_angle )
            
        #    if self.camera_preview:
        # print("image show")
        cv2.imshow("Image window", frame)
        cv2.waitKey(1)

    def calcTwist(self):
        # randomRun
        # value = random.randint(1,1000)
        # if value < 250:
        #    x = 0.2
        #    th = 0
        # elif value < 500:
        #    x = -0.2
        #    th = 0
        # elif value < 750:
        #    x = 0
        #    th = 1
        # elif value < 1000:
        #    x = 0
        #    th = -1
        # else:
        #    x = 0
        #    th = 0

        # run with scan data..
        print( self.front_distance )
        if self.front_distance > 0.45:
            x = 0.2
            th = 0
        else:
            x = 0
            th = (np.pi/4)

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(1) # change speed fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        # Main Loop --->
        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()

        # Main Loop <---
        
if __name__ == '__main__':
    rospy.init_node('seigo_run')
    bot = SeigoBot('Seigo')
    bot.strategy()

