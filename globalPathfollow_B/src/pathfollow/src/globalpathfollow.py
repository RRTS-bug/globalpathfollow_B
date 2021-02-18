#! /usr/bin/env python
# -*- coding: utf-8 -*-
import time
import rospy
import math
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
import roslib;roslib.load_manifest('pathfollow')
from pathfollow.msg import Traj
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
class Pathfollow(object):
    def __init__(self):
        self.dt = rospy.get_param('dt')
        self.vr = rospy.get_param('vr')
        self.drvr = rospy.get_param('drvr')
        self.k1 = rospy.get_param('k1')
        self.k2 = rospy.get_param('k2')
        self.theta1 = rospy.get_param('theta1')
        self.theta2 = rospy.get_param('theta2')
        self.sampleNum = rospy.get_param('sampleNum')
        self.maxLinearSpeed = rospy.get_param('maxLinearSpeed')  ##移动机器人最大行驶速度
        self.minLinearSpeed = rospy.get_param('minLinearSpeed') ##移动机器人最小行驶速度
        self.maxAcc = rospy.get_param('maxAcc')  ##移动机器人最大加速度
        self.minAcc = -self.maxAcc
        self.maxAngularSpeed = rospy.get_param('maxAngularSpeed')  ##移动机器人最大角速度
        self.minAngluarSpeed = -self.maxAngularSpeed
        self.lastlinearSpeed = 0

    def gettrajectory(self):
        path_topic = rospy.get_param('path_topic')
        msg = rospy.wait_for_message(path_topic, Traj)
        self.trajectory = [[msg.trajectoryX[i], msg.trajectoryY[i]] for i in range(len(msg.trajectoryX))]

    def getodom(self):
        odom_topic = rospy.get_param('odom_topic')
        odom = rospy.wait_for_message(odom_topic, Odometry)
        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        self.robotX = odom.pose.pose.position.x
        self.robotY = odom.pose.pose.position.y
        self.robotAngle = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        self.linearSpeed = odom.twist.twist.linear.x
        self.angularSpeed = odom.twist.twist.angular.z

    def getRobotOrd(self,robotX,robotY,robotAngle,targetX,targetY,targetAngle):
        self.sfX =   (targetX - robotX) * math.cos(robotAngle) + (targetY - robotY) * math.sin(robotAngle)
        self.sfY = - (targetX - robotX) * math.sin(robotAngle) + (targetY - robotY) * math.cos(robotAngle)
        self.sfAngle = targetAngle - robotAngle
        if self.sfAngle > math.pi:
            self.sfAngle = self.sfAngle - 2 * math.pi
        elif self.sfAngle <  - math.pi:
            self.sfAngle = 2 * math.pi + self.sfAngle

    def bspline(self):
        self.targetAngle = [0]
        self.angularRespect = [0]
        t = range(len(self.trajectory))
        trajectoryX = np.array(self.trajectory)[:, 0]
        trajectoryY = np.array(self.trajectory)[:, 1]
        x_tup = interpolate.splrep(t, trajectoryX, k=3)
        y_tup = interpolate.splrep(t, trajectoryY, k=3)
        ipl_t = np.linspace(0.0, len(self.trajectory) - 1, self.sampleNum)
        xBspline = interpolate.splev(ipl_t, x_tup)
        yBspline = interpolate.splev(ipl_t, y_tup)
        self.traj = [[xBspline[i], yBspline[i]] for i in range(len(xBspline))]
        self.targetAngle = self.targetAngle + [math.atan2(self.traj[i + 1][1] - self.traj[i][1] ,self.traj[i + 1][0] - self.traj[i][0]) for i in range(len(self.traj) - 1)]
        self.angularRespect = self.angularRespect + [(self.targetAngle[i + 1] - self.targetAngle[i]) / self.dt for i in range(len(self.targetAngle) - 1)]

    def slidefunc(self,targetIndex,pe1,pe2):
        derAlphaVr = self.sfY / (1 + math.pow(self.vr * self.sfY , 2))
        derAlphaYe = self.vr / (1 + math.pow(self.vr * self.sfY , 2))
        linearSpeedCon = self.sfY * self.angularSpeed + self.vr * math.cos(self.sfAngle) + self.k1 * pe1 / (math.fabs(pe1) + self.theta1)
        temp0 = self.angularRespect[targetIndex] + derAlphaVr * self.drvr + derAlphaYe * self.vr * math.sin(self.sfAngle) + self.k2 * pe2 / (math.fabs(pe2) + self.theta2)
        temp1 = 1 + derAlphaYe * self.sfX
        angularSpeedCon = temp0 / temp1
        return linearSpeedCon,angularSpeedCon

    def followmain(self):
        dist = [math.hypot(self.robotX - self.traj[i][0],self.robotY - self.traj[i][1]) for i in range(len(self.traj))]
        targetIndex = dist.index(min(dist)) + 1
        self.getRobotOrd(self.robotX,self.robotY,self.robotAngle,self.traj[targetIndex][0], self.traj[targetIndex][1], self.targetAngle[targetIndex])
        pe1 = self.sfX
        pe2 = self.sfAngle + math.atan2(self.vr * self.sfY,1)
        linearSpeedCOn, angularSpeedCon = self.slidefunc(targetIndex,pe1,pe2)
        return linearSpeedCOn, angularSpeedCon

    def lineaSpeedcheck(self, linearSpeed):
        acc = (linearSpeed - self.lastlinearSpeed) / self.dt
        if acc > self.maxAcc:
            acc = self.maxAcc
        elif acc < self.minAcc:
            acc = self.minAcc
        else:
            acc = acc
        linearSpeed = self.lastlinearSpeed + acc * self.dt
        if linearSpeed > self.maxLinearSpeed:
            linearSpeed = self.maxLinearSpeed
        elif linearSpeed < self.minLinearSpeed:
            linearSpeed = self.minLinearSpeed
        else:
            linearSpeed = linearSpeed
        self.lastlinearSpeed = linearSpeed
        return linearSpeed

        ##角速度检查，检查角速度控制命令是否在允许范围内

    def angularSpeedcheck(self, angularSpeed):
        if angularSpeed > self.maxAngularSpeed:
            angularSpeed = self.maxAngularSpeed
        elif angularSpeed < self.minAngluarSpeed:
            angularSpeed = self.minAngluarSpeed
        else:
            angularSpeed = angularSpeed
        return angularSpeed
def main():
    rospy.init_node('follow')
    pf = Pathfollow()
    cmd_vel_topic = rospy.get_param('cmd_vel_topic')
    cmd_pub = rospy.Publisher('cmd_vel_topic', Twist, queue_size=1)
    while not rospy.is_shutdown():
        pf.gettrajectory()
        pf.getodom()
        pf.bspline()
        linearSpeedCon, angularSpeedCon = pf.followmain()
        linearSpeed = pf.lineaSpeedcheck(linearSpeedCon)
        angularSpeed = pf.angularSpeedcheck(angularSpeedCon)
        twist = Twist()
        twist.linear.x = linearSpeed
        twist.angular.z = angularSpeed
        cmd_pub.publish(twist)
        
if __name__ == '__main__':
    main()