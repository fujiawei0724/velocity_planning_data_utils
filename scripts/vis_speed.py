#!/usr/bin/env python
from turtle import position
import rospy
import numpy as np
import time
import scipy.signal
import csv
from matplotlib import pyplot as plt
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class Recorder():

    def __init__(self):
        self.velocities_ = []
        self.tvs_ = []
        self.accelerations_ = []
        self.tas_ = []



    def velocity_callback(self, velocity_msg):
        print('Receive velocity data')
        self.velocities_.append(velocity_msg.data)
        self.tvs_.append(time.time())
    
    def acceleration_callback(self, acceleration_msg):
        print('Receive acceleration data')
        self.accelerations_.append(acceleration_msg.linear_acceleration.x)
        self.tas_.append(time.time())
    


    


if __name__ == "__main__":
    rospy.init_node("Recorder")
    re = Recorder()
    rospy.Subscriber("/planning_input/velocity", Float64, re.velocity_callback)
    rospy.Subscriber("/vehicle/imu/data_raw", Imu, re.acceleration_callback)

    while not rospy.is_shutdown():
        tvs = re.tvs_
        velocities = re.velocities_
        tas = re.tas_
        accelerations = re.accelerations_
        with open('./speed_profile.csv', 'w') as csv_file_speed:
            writer = csv.writer(csv_file_speed)
            for i in range(len(tvs)):
                writer.writerow([tvs[i], velocities[i]])
        with open('./acceleration_profile.csv', 'w') as csv_file_acceleration:
            writer = csv.writer(csv_file_acceleration)
            for i in range(len(tas)):
                writer.writerow([tas[i], accelerations[i]])

    rospy.spin()
