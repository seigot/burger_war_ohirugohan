#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tf
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from enum import Enum
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import json

# camera image 640*480
img_w = 640
img_h = 480

fieldWidth = 170
fieldHeight = 170
centerBoxWidth = 35
centerBoxHeight = 35
otherBoxWidth = 20
otherBoxHeight = 15
otherBoxDistance = 53

class ActMode(Enum):
    SEARCH = 1
    SNIPE  = 2
    ESCAPE = 3
    MOVE   = 4

class SeigoBot():
    myPosX = 0
    myPosY = -150
    myDirect = np.pi / 2
    lidarFig = plt.figure(figsize=(5,5))
    mapFig = plt.figure(figsize=(5,5))
    time_start = 0
    
    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # navigation publisher
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

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

 	# war status
	self.war_state = rospy.Subscriber("/red_bot/war_state", String, self.stateCallback)
        self.my_score = 0
        self.act_mode = ActMode.SEARCH

        # time
        self.time_start = time.time()

    def getElapsedTime(self):
        time_current = time.time()
        elapsed_time = time_current - self.time_start
        return elapsed_time
        
    # Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
    # Ref: https://github.com/hotic06/burger_war/blob/master/burger_war/scripts/navirun.py
    # do following command first.
    #   $ roslaunch burger_navigation multi_robot_navigation_run.launch
    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.name + "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data

        # visualize scan data with radar chart
        angles = np.linspace(0, 2 * np.pi, len(self.scan.ranges) + 1, endpoint=True)
        values = np.concatenate((self.scan.ranges, [self.scan.ranges[0]]))
        ax = self.lidarFig.add_subplot(111, polar=True)
        ax.cla()
        ax.plot(angles, values, 'o-')
        ax.fill(angles, values, alpha=0.25)
        ax.set_rlim(0, 3.5)
        # plt.pause(0.05) # draw

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

    def stateCallback(self, state):
        # print(state.data)
        dic = json.loads(state.data)
        tmp = int(dic["scores"]["r"])
	if tmp > self.my_score and self.act_mode == ActMode.SNIPE:
            self.act_mode = ActMode.SEARCH
        self.my_score = tmp
        print("---")
        print("my_sore", self.my_score)
        print("elapsed_time", self.getElapsedTime())
        
    def approachToMarker(self):
        x = 0
        th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def drawMap(self):
        myPosX = self.myPosX
        myPosY = self.myPosY
        myDirect = self.myDirect
        ax = self.mapFig.add_subplot(111)
        ax.cla()
        ax.set_xlim(-fieldWidth, fieldWidth)
        ax.set_ylim(-fieldHeight, fieldHeight)
        r1 = patches.Rectangle(xy=(-centerBoxWidth / 2,-centerBoxHeight / 2),
                               width=centerBoxWidth, height=centerBoxHeight,
                               ec='#000000', fill=False)
        r2 = patches.Rectangle(xy=(-otherBoxDistance - otherBoxWidth / 2,
                                   -otherBoxDistance - otherBoxHeight / 2),
                               width=otherBoxWidth, height=otherBoxHeight,
                               ec='#000000', fill=False)
        r3 = patches.Rectangle(xy=(+otherBoxDistance - otherBoxWidth / 2,
                                   -otherBoxDistance - otherBoxHeight / 2),
                               width=otherBoxWidth, height=otherBoxHeight,
                               ec='#000000', fill=False)
        r4 = patches.Rectangle(xy=(-otherBoxDistance - otherBoxWidth / 2,
                                   +otherBoxDistance - otherBoxHeight / 2),
                               width=otherBoxWidth, height=otherBoxHeight,
                               ec='#000000', fill=False)
        r5 = patches.Rectangle(xy=(+otherBoxDistance - otherBoxWidth / 2,
                                   +otherBoxDistance - otherBoxHeight / 2),
                               width=otherBoxWidth, height=otherBoxHeight,
                               ec='#000000', fill=False)
        a1 = patches.Arrow(myPosX, myPosY,
                           np.cos(myDirect) * 15, np.sin(myDirect) * 15, 10, ec='#0000FF')
        c1 = patches.Circle(xy=(myPosX, myPosY), radius=6, ec='#0000FF')
        ax.add_patch(r1)
        ax.add_patch(r2)
        ax.add_patch(r3)
        ax.add_patch(r4)
        ax.add_patch(r5)
        ax.add_patch(a1)
        ax.add_patch(c1)

        x = np.linspace(-fieldWidth, fieldWidth - 1, fieldWidth * 2)
        y = x + fieldHeight
        ax.plot(x, y, "r-")
        x = np.linspace(-fieldWidth, fieldWidth - 1, fieldWidth * 2)
        y = x - fieldHeight
        ax.plot(x, y, "r-")
        x = np.linspace(-fieldWidth, fieldWidth - 1, fieldWidth * 2)
        y = -x + fieldHeight
        ax.plot(x, y, "r-")
        x = np.linspace(-fieldWidth, fieldWidth - 1, fieldWidth * 2)
        y = -x - fieldHeight
        ax.plot(x, y, "r-")

    def getTwist(self, _x, _th):
        twist = Twist()
        x = _x
        th = _th
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th        
        return twist

    def func_init(self):
        print("func_init")
        twist = Twist()
        r = rospy.Rate(1) # change speed fps
        time.sleep(1.000) # wait for init complete

        print("!!!!!! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! !!!!!!")
        print("!                                                     !")
        print("! do following command first for navigation           !")
        print("! $ roslaunch burger_navigation multi_robot_navigation_run.launch !")
        print("! $ rosservice call /gazebo/reset_simulation {}       !")
        print("!                                                     !")
        print("!!!!!! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! !!!!!!")
        print("")
        
        ElapsedTime = self.getElapsedTime()
        print("ElapsedTime",ElapsedTime)

        return
    
    def func_search(self):
        print("func_search")
        twist = Twist()
        
        # 1: get 1st target
        self.setGoal(-0.9, 0.5, 0)

        # 2: get 2nd target
        self.setGoal(-0.9, -0.5, 0)

        # 3: get 3rd target
        self.setGoal(-0.8, 0.0, 0)
        self.setGoal(-0.4, 0.0, 0)
        # back
        twist = self.getTwist(-0.4, 0)
        self.vel_pub.publish(twist)
        time.sleep(1.0)
        
        # 4.0: check witch direction the enemy exists..
        # self.setGoal(0, 0.5, 0)
        # self.setGoal(0, 0.5, 3.1415)
        # self.setGoal(-0.5,0,-3.1415/2)

        # 4: get 4th target
        # turn
        #twist = self.getTwist(0.0, -3.1415/2)
        #self.vel_pub.publish(twist)
        #time.sleep(1.0)
        
        self.setGoal(0, -0.5, 0)
        # turn around
        twist = self.getTwist(0, 3.1415/1.5)
        self.vel_pub.publish(twist)
        time.sleep(3.0)

        self.act_mode = ActMode.SNIPE # transition to SNIPE

    def func_snipe(self):
        print("func_snipe")
        twist = Twist()
        
        # 1: go to snipe position
        self.setGoal(0, -0.5, 3.1415/2)
        twist = self.getTwist(-0.4, 0)
        self.vel_pub.publish(twist)
        time.sleep(2.0)

        # keep rotation
        cnt = 0
        while not rospy.is_shutdown():
            if cnt%2 == 0:
                twist = self.getTwist(0, 3.1415/2)
            else: # if cnt%2 == 1:
                twist = self.getTwist(0, -1*3.1415/2)
            self.vel_pub.publish(twist)
            time.sleep(1.5)
            cnt+=1
            
        #self.act_mode = ActMode.ESCAPE # transition to ESCAPE
        #self.act_mode = ActMode.MOVE # transition to MOVE
        return

    def func_escape(self):
        print("func_snipe")        

    def func_move(self):
        print("func_move")
        return

    def strategy(self):
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
     
        # Main Loop --->
        self.func_init()
        r = rospy.Rate(1) # change speed fps
        while not rospy.is_shutdown():
            print("act_mode: ", self.act_mode)

            if self.act_mode == ActMode.SEARCH:  # SEARCH
                self.func_search()
            elif self.act_mode == ActMode.SNIPE: # SNIPE
                self.func_snipe()
            elif self.act_mode == ActMode.ESCAPE: # ESCAPE
                self.func_escape()
            elif self.act_mode == ActMode.MOVE:   # MOVE
                self.func_move()
            else:
                print("act_mode: ", self.act_mode)
                
            r.sleep()
        # Main Loop <---
        
if __name__ == '__main__':
    #rospy.init_node('seigo_run')
    rospy.init_node('red_bot')
    bot = SeigoBot('red_bot')
    bot.strategy()

