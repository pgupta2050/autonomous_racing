#!/usr/bin/env python

from syslog import setlogmask
from turtle import end_fill
import rospy
import math
import time
import numpy as np
import matplotlib.pyplot as plt

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
        self.egoPose_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback, queue_size  = 15)
        self.comp1Pose_sub = rospy.Subscriber('/odom_Comp1', Odometry, self.comp1PosCallback, queue_size  = 10)
        self.traj_publisher = rospy.Publisher('/traj', JointTrajectory, queue_size=2)   
        self.timer = rospy.Timer(rospy.Duration(.1,0), self.timerCallback) #timer for every second
        
        DT = .1
        self.HORIZON = 20
        DSAFE = .5

        # Initial states
        x1_0 = 0
        y1_0 = 0
        v1_0 = 0
        theta1_0 = 0
        x2_0 = 0
        y2_0 = 0
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
        self.compx = 0
        self.compy = 3
        self.compv = 0
        self.comptheta = 0

        # X_0 = np.array([0, 1, 1, 0]).T
        X_ego_obsrvd = np.array([0, 0, 5, 0.75]).T
        xgoal = 10
        ygoal = 5
        self.goalList = [[10,20,30,40],
                         [5,10,20,30]]

        # self.X_0 = X_0
        self.xgoal = xgoal
        self.ygoal = ygoal
        self.X_ego_obsrvd = X_ego_obsrvd

        self.egotraj = JointTrajectory()

        self.X0List = np.zeros((20,4))
        self.stepCount = 0 # step np. inside the segment (row number for saving comp traj)
        self.segmentCount = 0 # which segment from ilqr being tracked
        self.solve_ilqr = 0 # flag for when the ilqr needs to be solved

    def odomCallback(self, msg):
        v = msg.twist.twist.linear.x
        y = msg.pose.pose.position.y
        x = msg.pose.pose.position.x
        phi = msg.pose.pose.orientation.z
        self.X_ego_obsrvd = np.array([x,y,v,phi]).T
        
        # rospy.loginfo("Received low level odometry data for this step")

        if (msg.pose.pose.position.z == 1):
            self.v1_0 = msg.twist.twist.linear.x
            self.y1_0 = msg.pose.pose.position.y
            self.x1_0 = msg.pose.pose.position.x
            self.theta1_0 = msg.pose.pose.orientation.z

            # tracked one segment. update competitor states for highlevel
            self.v2_0 = self.compv
            self.y2_0 = self.compy
            self.x2_0 = self.compx
            self.theta2_0 = self.comptheta

            # this means new goal to be set
            # self.xgoal = 20
            # self.ygoal = 10
            self.updateGoal(self.segmentCount)
            rospy.loginfo("updated goal for segment : %d", self.segmentCount)

            # reset step count for new segment
            self.stepCount = 0
            self.segmentCount = self.segmentCount + 1
            self.solve_ilqr = 1

        # Update competitor state
        self.X_0 = np.array([self.compx,self.compy,self.compv,self.comptheta]).T
        X, U = self.compAgent1.MPCOpt(self.X_0, self.xgoal2, self.ygoal2, self.X_ego_obsrvd)
        self.compx = X[0,1]
        self.compy = X[1,1]
        self.compv = X[2,1]
        self.comptheta = X[3,1]

        # Update step counter
        self.X0List[self.stepCount,:] = self.X_0
        self.stepCount = self.stepCount + 1

        print("---------Segment: ",self.segmentCount -1 ," --------------------")
        print("---------Step of segment: ",self.stepCount," --------------------")
        print("Ego's state: ", self.X_ego_obsrvd) #, self.compx,self.compy,self.compv,self.comptheta)
        print("Competitor state: ", self.X_0)
        print("Com traj is : ", self.X0List)
        print("----------------------------------------------------------")
        
    def comp1PosCallback(self, msg):
        self.v2_0 = msg.twist.twist.linear.x
        self.y2_0 = msg.pose.pose.position.y
        self.x2_0 = msg.pose.pose.position.x
        self.theta2_0 = msg.pose.pose.orientation.z

    def updateGoal(self,i):
        self.xgoal1 = self.goalList[0][i]
        self.ygoal1 = self.goalList[1][i]
        self.xgoal2 = self.goalList[0][i]
        self.ygoal2 = self.goalList[1][i]

    def Shutdown(self):
        rospy.loginfo("Killing the high level node")
        time.sleep(0.2)

    def timerCallback(self,timer):
        time_begin = rospy.Time.now()

        if self.solve_ilqr == 1:
                        
            # if self.segmentCount == 0:
            #     print("segment is the first one count =0")
            #     print(self.x1_0, self.y1_0, self.v1_0, self.theta1_0, self.x2_0, self.y2_0, self.v2_0, self.theta2_0, \
            #                     self.xgoal1, self.ygoal1, self.xgoal2, self.ygoal2, self.a1_0, self.omega1_0, self.a2_0, self.omega2_0)
            #     Xsol, _ = self.hlplanner.solve_ilqr(self.x1_0, self.y1_0, self.v1_0, self.theta1_0, 0, 3, 0, 0, \
            #                      self.xgoal1, self.ygoal1, self.xgoal2, self.ygoal2, self.a1_0, self.omega1_0, self.a2_0, self.omega2_0)
            # else:
            print(self.x1_0, self.y1_0, self.v1_0, self.theta1_0, self.x2_0, self.y2_0, self.v2_0, self.theta2_0, \
                            self.xgoal1, self.ygoal1, self.xgoal2, self.ygoal2, self.a1_0, self.omega1_0, self.a2_0, self.omega2_0)
            Xsol, _ = self.hlplanner.solve_ilqr(self.x1_0, self.y1_0, self.v1_0, self.theta1_0, self.x2_0, self.y2_0, self.v2_0, self.theta2_0, \
                            self.xgoal1, self.ygoal1, self.xgoal2, self.ygoal2, self.a1_0, self.omega1_0, self.a2_0, self.omega2_0)

            time_end = rospy.Time.now()
            duration = time_end - time_begin
            rospy.loginfo("ilqr call+comutation for " + str(duration.to_sec()) + " secs")
            # print("Xsol from ilqr is: \n", Xsol)
        
            pointsList = []
            # xsol = 12x20
            # publish only the 4 ego states over the horizon.
            for i in range(0,self.HORIZON+1):
                p = JointTrajectoryPoint()
                p.positions = [Xsol[i][0], Xsol[i][1], Xsol[i][3]]
                p.velocities = [Xsol[i][2]]
                pointsList.append(p)        

            self.egotraj.joint_names = ['horizon trajectories 1-20']
            self.egotraj.header.stamp = rospy.Time.now()
            self.egotraj.points = pointsList
            self.traj_publisher.publish(self.egotraj)
        
            np.savetxt("comptraj.txt",self.X0List)

            self.solve_ilqr = 0 # reset the flag

        # rospy.loginfo("In timerCallback")

if __name__ == '__main__':
     try:
        rospy.init_node('high_level_node', anonymous=False)
        hl = Highlevel()
        rospy.loginfo("hl Ready")

        while not rospy.is_shutdown():
            time.sleep(0.001)

     except rospy.ROSInterruptException:
         
         pass