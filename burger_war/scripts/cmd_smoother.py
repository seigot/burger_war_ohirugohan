#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import rospy
from geometry_msgs.msg import Twist


class CmdSmoother:

    def __init__(self):
        rospy.Subscriber("/move_base/cmd_vel", Twist, self.get_twist)
        self.pub = rospy.Publisher("cmd_vel", Twist)
        self.vx_max = 0.22
        self.omega_max = 2.75
        self.acc_x_max = 2.5
        self.acc_omega_max = 3.2
        self.pre_cmd = Twist()
        self.pre_time = time.time()

    def get_twist(self, target_cmd):
        current_time = time.time()
        limit_cmd_vel = Twist()
        t = current_time-self.pre_time
        if self.pre_cmd.linear.x < target_cmd.linear.x:
            sign = 1.0
        elif self.pre_cmd.linear.x > target_cmd.linear.x:
            sign = -1.0
        else:
            sign = 0
        limit_cmd_vel.linear.x = self.pre_cmd.linear.x + sign*self.acc_x_max*t
        if limit_cmd_vel.linear.x < target_cmd.linear.x:
            target_cmd.linear.x = limit_cmd_vel.linear.x

        if self.pre_cmd.angular.z < target_cmd.angular.z:
            sign = 1.0
        elif self.pre_cmd.angular.z > target_cmd.angular.z:
            sign = -1.0
        else:
            sign = 0
        limit_cmd_vel.angular.z = self.pre_cmd.angular.z + sign*self.acc_x_max*t
        if limit_cmd_vel.angular.z < target_cmd.angular.z:
            target_cmd.angular.z = limit_cmd_vel.angular.z
        
        self.pub.publish(target_cmd)
        self.pre_time = time.time()
        self.pre_cmd = target_cmd

    def start(self):
        rospy.init_node("cmd_smoother")
        rospy.spin()

if __name__ == "__main__":
    node = CmdSmoother()
    node.start()
