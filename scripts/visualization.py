#!/usr/bin/env python
from turtle import position
import rospy
import numpy as np
import time
import scipy.signal
from matplotlib import pyplot as plt
from std_msgs.msg import Float64


class Recorder():

    def __init__(self):
        self.velocities_ = []
        self.steers_ = []



    def velocity_callback(self, velocity_msg):
        self.velocities_.append(velocity_msg.data)
        if len(self.velocities_) > 100:
            self.velocities_.pop(0)
    
    def steer_callback(self, steer_msg):
        self.steers_.append(steer_msg.data)
        if len(self.steers_) > 100:
            self.steers_.pop(0)
    

        

    def draw(self):
        velocity_valid_length = len(self.velocities_)
        steer_valid_length = len(self.steers_)
        
        plt.clf()

        ax = fig.add_subplot(2, 1, 1)
        ax.plot(np.arange(0, velocity_valid_length, 1), self.velocities_[:velocity_valid_length], c='r', linewidth=1.5)
        ax.set_ylim(0.0, 18.0)
        ax.set_title('$Velocity (m/s)$')
        ax.set_xlabel('$Time$')
        ax.set_xticks([])


        bx = fig.add_subplot(2, 1, 2)
        bx.plot(np.arange(0, steer_valid_length, 1), np.array(self.steers_[:steer_valid_length]), c='g', linewidth=1.5)
        bx.set_ylim(-0.001, 0.001)
        bx.set_title('$Steer (rad)$')
        bx.set_xlabel('$Time$')
        bx.set_xticks([])
        plt.tight_layout()


        plt.pause(0.01)


if __name__ == "__main__":
    rospy.init_node("Recorder")
    vis = Recorder()
    fig = plt.figure(figsize=(5, 8), facecolor='white', edgecolor='black')
    plt.ion()
    rospy.Subscriber("/planning_input/velocity", Float64, vis.velocity_callback)
    rospy.Subscriber("/planning_input/steer", Float64, vis.steer_callback)

    while not rospy.is_shutdown():
        vis.draw()
    rospy.spin()

    



