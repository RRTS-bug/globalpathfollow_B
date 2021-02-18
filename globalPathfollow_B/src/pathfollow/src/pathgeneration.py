#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import roslib;roslib.load_manifest('pathfollow')  
from pathfollow.msg import Traj
import numpy as np
def main():
    rospy.init_node('path', anonymous=True) 
    pub = rospy.Publisher('trajectory',Traj)
    r = rospy.Rate(40) # 1000hz  
    while not rospy.is_shutdown():  
        x = np.linspace(np.pi, 10 * np.pi, 20)
        y = 2 * np.sin(x) + 3
        x.tolist()
        y.tolist()
        tr = [[x[i],y[i]] for i in range(len(x))]
        traj = Traj()
        traj.trajectoryX = x
        traj.trajectoryY = y
        pub.publish(traj)
        r.sleep
if __name__ == '__main__':
    main()
