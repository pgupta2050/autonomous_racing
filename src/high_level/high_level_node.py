#!/usr/bin/env python

from syslog import setlogmask
from turtle import end_fill
import rospy
import math
import time
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
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
        self.comp1odom_pub = rospy.Publisher('/odom_comp1', Odometry, queue_size  = 10)
        self.traj_publisher = rospy.Publisher('/traj', JointTrajectory, queue_size=2)   
        self.timer = rospy.Timer(rospy.Duration(.1,0), self.timerCallback) # timer for every second
        
        DT = .1
        self.HORIZON = 10
        DSAFE = .5

        # Initial states
        x1_0 = 0
        y1_0 = -2
        v1_0 = 2
        theta1_0 = 0
        x2_0 = 0
        y2_0 = 2
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
        self.compy = 2
        self.compv = 0
        self.comptheta = 0
        self.compa = 0
        self.compomega = 0

        # X_0 = np.array([0, 1, 1, 0]).T
        # X_ego_obsrvd = np.array([0, 0, 5, 0.75]).T
        X_ego_obsrvd = np.array([0, -2, 0, 0]).T
        xgoal = 10
        ygoal = 5
        self.goalList = [[10,20,30,40,50],
                         [5,10,20,30,40]]
        # self.goalList = [[0 ,3.0100, 11.6900, 25.0000, 41.3100, 58.6800, 75.0000, 88.3000, 96.9800,100.0000],
        #                  [0, 17.1000, 32.1300, 43.3000, 49.2400, 49.2400, 43.3000, 32.1300, 17.1000, 0]]
        # self.goalList = [[0, 17.1010, 32.1394,   43.3013,   49.2404,   49.2404,   43.3013,   32.1394,   17.1010,         0],
                        # [0, -3.0154,  -11.6978,  -25.0000,  -41.3176,  -58.6824,  -75.0000,  -88.3022,  -96.9846, -100.0000]]
        self.goalList = [[0 , 8.2297, 16.2350, 23.7974, 30.7106, 36.7862, 41.8583, 45.7887, 48.4700, 49.8292, 49.8292, 48.4700, 45.7887, 41.8583, 36.7862, 30.7106, 23.7974, 16.2350, 8.2297, 0, -8],
                        [0, -0.6819, -2.7091, -6.0263,-10.5430,-16.1359,-22.6526,-29.9152,-37.7257,-45.8710,-54.1290,-62.2743,-70.0848,-77.3474,-83.8641,-89.4570,-93.9737,-97.2909,-99.3181, -100.0000, -100]]
        # self.X_0 = X_0
        self.xgoal = xgoal
        self.ygoal = ygoal
        self.X_ego_obsrvd = X_ego_obsrvd

        self.egotraj = JointTrajectory()
        self.compOdom = Odometry()
        
        self.compTraj = np.zeros((500,4))
        self.X0List = np.zeros((16,4))

        self.stepCount = 0 # step np. inside the segment (row number for saving comp traj)
        self.segmentCount = 0 # which segment from ilqr being tracked
        self.solve_ilqr = 0 # flag for when the ilqr needs to be solved

    def odomCallback(self, msg):
        v = msg.twist.twist.linear.x
        y = msg.pose.pose.position.y
        x = msg.pose.pose.position.x
        phi = msg.pose.pose.orientation.z
        self.X_ego_obsrvd = np.array([x,y,v,phi]).T
        
        if (msg.pose.pose.position.z == 1):
            self.v1_0 = msg.twist.twist.linear.x
            self.y1_0 = msg.pose.pose.position.y
            self.x1_0 = msg.pose.pose.position.x
            self.theta1_0 = msg.pose.pose.orientation.z
            # self.a1_0 = msg.twist.twist.angular.x #acc
            # self.omega1_0 = msg.twist.twist.angular.z #steer rate

            # tracked one segment. update competitor states for highlevel
            self.v2_0 = self.compv
            self.y2_0 = self.compy
            self.x2_0 = self.compx
            self.theta2_0 = self.comptheta
            # self.a2_0 = self.compa
            # self.omega2_0 = self.compomega

            # this means new goal to be set for new plan
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
        self.compa = U[0,1]
        self.compomega = U[1,1]
        self.compOdom.pose.pose.position.x = self.compx
        self.compOdom.pose.pose.position.y = self.compy
        self.compOdom.pose.pose.orientation.z = self.comptheta
        self.compOdom.twist.twist.linear.x = self.compv
        self.compOdom.twist.twist.angular.z = self.compomega
        self.compOdom.twist.twist.angular.x = self.compa
        # self.comp1odom_pub.publish(self.compOdom)

        # save competitor trajectory
        self.X0List[self.stepCount,:] = self.X_0
        segk = (self.segmentCount-1)*16
        self.compTraj[segk + self.stepCount,:] = self.X_0

        # Update step counter
        self.stepCount = self.stepCount + 1

        print("---------Segment: ",self.segmentCount -1 ," --------------------")
        print("---------Step of segment: ",self.stepCount-1," --------------------")
        print("Ego's state: ", self.X_ego_obsrvd) #, self.compx,self.compy,self.compv,self.comptheta)
        print("Competitor state: ", self.X_0)
        # print("Com traj is : ", self.X0List)
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
        self.comp1odom_pub.publish(self.compOdom)
        if self.solve_ilqr == 1:
                        
            Xsol, _ = self.hlplanner.solve_ilqr(self.x1_0, self.y1_0, self.v1_0, self.theta1_0, self.x2_0, self.y2_0, self.v2_0, self.theta2_0, \
                            self.xgoal1, self.ygoal1, self.xgoal2, self.ygoal2, self.a1_0, self.omega1_0, self.a2_0, self.omega2_0)

            time_end = rospy.Time.now()
            duration = time_end - time_begin
            rospy.loginfo("ilqr call+comutation for " + str(duration.to_sec()) + " secs")
        
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
        
            np.savetxt("comptraj.txt",self.compTraj)

            self.solve_ilqr = 0 # reset the flag

if __name__ == '__main__':
     try:
        rospy.init_node('high_level_node', anonymous=False)
        hl = Highlevel()
        rospy.loginfo("hl Ready")

        while not rospy.is_shutdown():
            time.sleep(0.001)

     except rospy.ROSInterruptException:
         
         pass