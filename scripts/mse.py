#!/usr/bin/env python
from rosdep2 import RosdepLookup
import rospy
import rospkg
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger, TriggerResponse
import time
import math
import csv

class Recorder():
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
        self.rospack = rospkg.RosPack()

    
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
        if self.x_odom != None:  
            rmse = math.sqrt(math.pow((self.x_odom - self.x_pose),2) + math.pow((self.y_odom - self.y_pose),2))
            self.rmses.append(rmse)
            self.rmse_times.append(time.time() - self.init_time)

    def write2csv(self,datalist,file = "no_algo.csv"):
        with open(file,'a') as f:
            cw  = csv.writer(f)
            cw.writerow(datalist) 
    
    def reinitial_callback(self,req):

        # rospy.signal_shutdown("servive signal stop record")#initial shutdown process
        del self.rmses[:]
        del self.rmse_times[:]
        self.init_time = time.time()
        rospy.loginfo("record reinitial!")

        return TriggerResponse(1, "record reinitial!")

    def success_shutdown_callback(self,req):

        self.write2csv(self.rmses,file=self.rospack.get_path("sim_localization")+"/no_algo.csv")
        rospy.loginfo("stop record!")
        return TriggerResponse(1, "stop record!")
        





if __name__ == "__main__":
    rospy.init_node("Recorder")
    vis = Recorder()

    # gazebo's odom, without nosie
    rospy.Subscriber("odom", Odometry, vis.odom_callback)
    rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,vis.amcl_pose_callback)
    rospy.Service('/reinitial_record',Trigger,vis.reinitial_callback)
    rospy.Service('/success_stop_record',Trigger,vis.success_shutdown_callback)
    rospy.spin()
    if rospy.is_shutdown():
        pass
        # vis.write2csv(vis.rmses)
        # plt.plot(vis.rmse_times, vis.rmses, color="b", label="/odom")
        # plt.show()


