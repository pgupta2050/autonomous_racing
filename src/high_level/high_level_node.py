#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
from numpy.linalg import inv
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
import os
import csv

import pdgplanner
import aiagent

class Highlevel():

    def __init__(self):
        rospy.on_shutdown(self.Shutdown)
        loop_rate = 1000
        self.rate = rospy.Rate(loop_rate)
        self.position_feedback = rospy.Subscriber('/odom', Odometry, self.odomCallback, queue_size  = 2)
        self.traj_publisher = rospy.Publisher('/traj', MultiDOFJointTrajectoryPoint, queue_size=2)   
        self.timer = rospy.Timer(0.1,self.timerCallback)
        DT = .1
        HORIZON = 20
        DSAFE = 1

        self.hlplanner = pdgplanner.PDGILQR(DT,HORIZON,DSAFE)
        self.x1_0 = x1_0
        self.y1_0 = y1_0
        self.v1_0 = v1_0
        self.theta1_0 = theta1_0
        self.x2_0 = x2_0
        self.y2_0 = y2_0
        self.v2_0 = v2_0
        self.theta2_0 = theta2_0
        self.xgoal1 = xgoal1
        self.ygoal1 = ygoal1
        self.xgoal2 = xgoal2
        self.ygoal2 = ygoal2
        self.a1_0 = a1_0
        self.omega1_0 = omega1_0
        self.a2_0 = a2_0
        self.omega2_0 = omega2_0

        self.compAgent = aiagent.Competitor1()

    def odomCallback(self, msg):
        self.v1_0 = msg.Twist.Twist.X
        self.y1_0 = msg.Pose.Pose.Position.Y
        self.x1_0 = msg.Pose.Pose.Position.X
        self.theta1_0 = msg.Pose.Pose.Orientation.Z
        

    def Shutdown(self):
            rospy.loginfo("Cleaning out the high level node")
            time.sleep(0.2)

    def timerCallback(self):
        self.hlplanner.solve_ilqr(self.x1_0, self.y1_0, self.v1_0, self.theta1_0, self.x2_0, self.y2_0, self.v2_0, self.theta2_0, self.xgoal1, self.ygoal1, self.xgoal2, self.ygoal2, self.a1_0, self.omega1_0, self.a2_0, self.omega2_0)

    
if __name__ == '__main__':
     try:
        rospy.init_node('high_level_node', anonymous=False)
        hl = Highlevel()
        rospy.loginfo("hl Ready")

        while not rospy.is_shutdown():
            time.sleep(0.001)
            hl.Send()

     except rospy.ROSInterruptException:
         pass





    


