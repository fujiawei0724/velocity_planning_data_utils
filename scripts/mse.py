#!/usr/bin/env python
from rosdep2 import RosdepLookup
import rospy
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import math

class Visualiser():
    #although variates below is belong to the whole class , we only use them once anyway
    odom_time, cb_time = 0, 0
    x_odom = None
    y_odom = None
    x_odoms = []
    y_odoms = []
    rmses = []
    rmse_times = []

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.init_time = time.time()

    
    def odom_callback(self, data):#fps:30hz
        rospy.loginfo("odom pose: x:%0.6f , y:%0.6f",data.pose.pose.position.x,data.pose.pose.position.y)
        self.x_odom = data.pose.pose.position.x
        self.y_odom = data.pose.pose.position.y        
        self.x_odoms.append(self.x_odom)
        self.y_odoms.append(self.y_odom)
        self.odom_time = time.time()
    def amcl_pose_callback(self,data):#fps:1hz
        rospy.loginfo("amcl pose: x:%0.6f , y:%0.6f",data.pose.pose.position.x,data.pose.pose.position.y)
        self.x_pose = data.pose.pose.position.x
        self.y_pose = data.pose.pose.position.y  
        rmse = math.sqrt(math.pow((self.x_odom - self.x_pose),2) + math.pow((self.y_odom - self.y_pose),2))
        self.rmses.append(rmse)
        self.rmse_times.append(time.time() - self.init_time)
        





if __name__ == "__main__":
    rospy.init_node("visualiser")
    vis = Visualiser()

    # gazebo's odom, without nosie
    rospy.Subscriber("odom", Odometry, vis.odom_callback)
    rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,vis.amcl_pose_callback)
    rospy.spin()
    if rospy.is_shutdown():
        plt.plot(vis.rmse_times, vis.rmses, color="b", label="/odom")
        plt.show()


