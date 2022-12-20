#!/usr/bin/env python
from turtle import position, st
import rospy
import time
import random
import numpy as np
from matplotlib import pyplot as plt
from ibeo_lidar_msgs.msg import object_filter_data

class NoiseObstacleGenerator:
    def __init__(self,
                 length_coeff_ratio=0.2,
                 width_coeff_ratio=0.2,
                 velocity_coeff_ratio=0.1,
                 orientation_coeff_ratio=0.1,
                 position_x_coeff_abs=0.2,
                 position_y_coeff_abs=0.2):
        self.length_coeff_ratio = length_coeff_ratio
        self.width_coeff_ratio = width_coeff_ratio
        self.velocity_coeff_ratio = velocity_coeff_ratio
        self.orientation_coeff_ratio = orientation_coeff_ratio
        self.position_x_coeff_abs = position_x_coeff_abs
        self.position_y_coeff_abs = position_y_coeff_abs

        self.noisy_obstacle_publisher = rospy.Publisher('/perception/obstacle_with_noise', object_filter_data, queue_size=1)
    
    def obstacle_callback(self, obstacle_msg):
        processed_obstacle_msg = obstacle_msg
        print('[NoiseObstacleGenerator] Current filtered obstalces number: {}'.format(len(processed_obstacle_msg.objects)))
        for obs_info in processed_obstacle_msg.objects:
            obs_info.length += random.gauss(0.0, self.length_coeff_ratio*obs_info.length)
            obs_info.width += random.gauss(0.0, self.width_coeff_ratio*obs_info.width)
            obs_info.center.x += random.gauss(0.0, self.position_x_coeff_abs)
            obs_info.center.y += random.gauss(0.0, self.position_y_coeff_abs)
            obs_info.orientation += random.gauss(0.0, self.orientation_coeff_ratio*obs_info.orientation)
            obs_info.v += random.gauss(0.0, self.velocity_coeff_ratio*obs_info.v)
            obs_info.velocity.x = obs_info.v * np.cos(obs_info.orientation)
            obs_info.velocity.y = obs_info.v * np.sin(obs_info.orientation)
        self.noisy_obstacle_publisher.publish(processed_obstacle_msg)

if __name__ == '__main__':
    rospy.init_node('noisy_obstacle_generator_node')
    noise_obstacle_generator = NoiseObstacleGenerator()
    rospy.Subscriber('/perception/objects', object_filter_data, noise_obstacle_generator.obstacle_callback)
    rospy.spin()



        


        