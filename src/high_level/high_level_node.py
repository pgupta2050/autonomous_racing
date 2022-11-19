#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import pdgplanner
import aiagent

class Highlevel():

    def __init__(self):
        rospy.on_shutdown(self.Shutdown)
        loop_rate = 10
        self.rate = rospy.Rate(loop_rate)
        self.egoPose_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback, queue_size  = 2)
        self.comp1Pose_sub = rospy.Subscriber('/odom_Comp1', Odometry, self.comp1PosCallback, queue_size  = 2)
        self.traj_publisher = rospy.Publisher('/traj', JointTrajectory, queue_size=2)   
        self.timer = rospy.Timer(rospy.Duration(.1,0), self.timerCallback) #timer for every second
        
        DT = .1
        self.HORIZON = 20
        DSAFE = 1

        # Initial states
        x1_0 = 0
        y1_0 = 0
        v1_0 = 0
        theta1_0 = 0
        x2_0 = 0
        y2_0 = 3
        v2_0 = 0
        theta2_0 = 0
        # Initial augemnted control-states
        a1_0 = 0
        omega1_0 = 0
        a2_0 = 0
        omega2_0 = 0
        
        # Goal
        xgoal1 = 10
        ygoal1 = 5
        xgoal2 = 10
        ygoal2 = 5

        self.hlplanner = pdgplanner.PDGILQR(DT,self.HORIZON,DSAFE)
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

        self.compAgent1 = aiagent.Competitor1()
        
        X_0 = np.array([0, 2, 1, 0]).T
        X_ego_obsrvd = np.array([0, 0, 5, 0.75]).T
        xgoal = ygoal = 5

        self.X_0 = X_0
        self.xgoal = xgoal
        self.ygoal = ygoal
        self.X_ego_obsrvd = X_ego_obsrvd

        self.egotraj = JointTrajectory()

    def odomCallback(self, msg):
        self.v1_0 = msg.twist.twist.x
        self.y1_0 = msg.pose.pose.position.y
        self.x1_0 = msg.pose.pose.position.x
        self.theta1_0 = msg.pose.pose.orientation.z
        
    def comp1PosCallback(self, msg):
        self.v2_0 = msg.twist.twist.x
        self.y2_0 = msg.pose.pose.position.y
        self.x2_0 = msg.pose.pose.position.x
        self.theta2_0 = msg.pose.pose.orientation.z

    def Shutdown(self):
        rospy.loginfo("Killing the high level node")
        time.sleep(0.2)

    def timerCallback(self,timer):
        # print("Aiyyo")
        time_begin = rospy.Time.now()
    
        Xsol, _ = self.hlplanner.solve_ilqr(self.x1_0, self.y1_0, self.v1_0, self.theta1_0, self.x2_0, self.y2_0, self.v2_0, self.theta2_0, \
                                self.xgoal1, self.ygoal1, self.xgoal2, self.ygoal2, self.a1_0, self.omega1_0, self.a2_0, self.omega2_0)
        X, U = self.compAgent1.MPCOpt(self.X_0, self.xgoal, self.ygoal, self.X_ego_obsrvd)
        xego, yego = self.compAgent1.const_vel_model(self.X_ego_obsrvd)

        time_end = rospy.Time.now()
        duration = time_end - time_begin
        rospy.loginfo("ilqr call+comutation for " + str(duration.to_sec()) + " secs")

        pointsList = []
        # xsol = 12x20
        # publish only the 4 ego states over the horizon.
        for i in range(0,self.HORIZON+1):
            p = JointTrajectoryPoint()
            # print(Xsol)
            p.positions = [Xsol[i][0], Xsol[i][1], Xsol[i][3]]
            p.velocities = [Xsol[i][2]]
            pointsList.append(p)

        self.egotraj.joint_names = ['horizon trajectories 1-20']
        self.egotraj.header.stamp = rospy.Time.now()
        self.egotraj.points = pointsList
        self.traj_publisher.publish(self.egotraj)
    
if __name__ == '__main__':
     try:
        rospy.init_node('high_level_node', anonymous=False)
        hl = Highlevel()
        rospy.loginfo("hl Ready")

        while not rospy.is_shutdown():
            time.sleep(0.001)

     except rospy.ROSInterruptException:
         pass