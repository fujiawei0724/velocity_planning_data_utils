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

        self.curvatures_ = []
        self.t_curvature_s_ = []

        self.min_obstacle_distances_ = []
        self.t_obs_dis_s_ = []





    def velocity_callback(self, velocity_msg):
        # Velocity information is provided by the vehicle simulator
        print('Receive velocity data')
        self.velocities_.append(velocity_msg.data)
        self.tvs_.append(time.time())
    
    def acceleration_callback(self, acceleration_msg):
        # Acceleration information is provided by the IMU simulator
        print('Receive acceleration data')
        self.accelerations_.append(acceleration_msg.linear_acceleration.x)
        self.tas_.append(time.time())
    
    def curvature_callback(self, steer_msg):
        # Curvature information is converted from the front wheel angle from bicycle model
        print('Receive curvaturte data')
        self.curvatures_.append(np.tan(steer_msg.data) / 2.85)
        self.t_curvature_s_.append(time.time())

    def obstacle_distance_callback(self, distance_msg):
        # Distance information is provided by the motion planning module
        print('Receive obstacle distance data')
        self.min_obstacle_distances_.append(distance_msg.data)
        self.t_obs_dis_s_.append(time.time())
    


    


if __name__ == "__main__":
    rospy.init_node("data_recorder_node")
    re = Recorder()
    rospy.Subscriber("/planning_input/velocity", Float64, re.velocity_callback)
    rospy.Subscriber("/vehicle/imu/data_raw", Imu, re.acceleration_callback)
    rospy.Subscriber("/planning_input/steer", Float64, re.curvature_callback)
    rospy.Subscriber("/motion_planning/obstacle_distance", Float64, re.obstacle_distance_callback)

    start_time = time.time()

    while not rospy.is_shutdown():
        tvs = re.tvs_
        velocities = re.velocities_
        tas = re.tas_
        accelerations = re.accelerations_
        t_curvature_s = re.t_curvature_s_
        curvatures = re.curvatures_
        t_obs_dis_s = re.t_obs_dis_s_
        obstacle_distances = re.min_obstacle_distances_

        if time.time() - start_time < 20.0:
            continue

        start_time = time.time()

        with open('./speed_profile.csv', 'w') as csv_file_speed:
            writer = csv.writer(csv_file_speed)
            for i in range(len(tvs)):
                writer.writerow([tvs[i], velocities[i]])
        with open('./acceleration_profile.csv', 'w') as csv_file_acceleration:
            writer = csv.writer(csv_file_acceleration)
            for i in range(len(tas)):
                writer.writerow([tas[i], accelerations[i]])
        with open('./curvature_profile.csv', 'w') as csv_file_curvature:
            writer = csv.writer(csv_file_curvature)
            for i in range(len(t_curvature_s)):
                writer.writerow([t_curvature_s[i], curvatures[i]])
        with open('./obstacle_distances.csv', 'w') as csv_file_obs_distance:
            writer = csv.writer(csv_file_obs_distance)
            for i in range(len(t_obs_dis_s)):
                writer.writerow([t_obs_dis_s[i], obstacle_distances[i]])


    rospy.spin()
