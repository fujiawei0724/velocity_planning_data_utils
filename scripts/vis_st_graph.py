#!/usr/bin/env python
from turtle import position, st
import rospy
import numpy as np
import time
import scipy.signal
import csv
from shapely.geometry import Polygon
from collections import namedtuple
from matplotlib import pyplot as plt
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from data_visualization_msg.msg import StGraph

StGraphInfo = namedtuple('StGraphInfo', ['initial_corridors', 'enhanced_corridors', 'occupied_areas', 'velocity_profile_info', 'current_state_name'])
DRAW_DT = 0.2


class StGraphRecorder:

    def __init__(self):
        self.forward_st_graph_info_ = None
        self.left_st_graph_info_ = None
        self.right_st_graph_info_ = None
        self.forward_last_update_time_ = None
        self.turn_left_last_update_time_ = None
        self.turn_right_last_update_time_ = None
    

    def forward_st_graph_callback(self, st_graph_msg):
        current_st_graph_info = StGraphInfo._make([st_graph_msg.initial_corridors, st_graph_msg.enhanced_corridors, st_graph_msg.occupied_areas, st_graph_msg.velocity_info, st_graph_msg.state_name])
        if st_graph_msg.state_name == 'FORWARD':
            self.forward_st_graph_info_ = current_st_graph_info
            self.forward_last_update_time_ = time.time()
    
    def turn_left_st_graph_callback(self, st_graph_msg):
        current_st_graph_info = StGraphInfo._make([st_graph_msg.initial_corridors, st_graph_msg.enhanced_corridors, st_graph_msg.occupied_areas, st_graph_msg.velocity_info, st_graph_msg.state_name])
        if st_graph_msg.state_name == 'TURN_LEFT':
            self.left_st_graph_info_ = current_st_graph_info
            self.turn_left_last_update_time_ = time.time()


    def turn_right_st_graph_callback(self, st_graph_msg):
        current_st_graph_info = StGraphInfo._make([st_graph_msg.initial_corridors, st_graph_msg.enhanced_corridors, st_graph_msg.occupied_areas, st_graph_msg.velocity_info, st_graph_msg.state_name])
        if st_graph_msg.state_name == 'TURN_RIGHT':
            self.right_st_graph_info_ = current_st_graph_info
            self.turn_right_last_update_time_ = time.time()


class Cube2D:
    def __init__(self, t_start, t_end, s_start, s_end):
        self.t_start_ = t_start
        self.t_end_ = t_end
        self.s_start_ = s_start
        self.s_end_ = s_end
        self.vertex_ = np.array([[t_start, s_start], [t_start, s_end], [t_end, s_end], [t_end, s_start]])


class StGraphVisualizer:
    @staticmethod
    def supply_st_fig(st_info, corre_ax, last_update_timestamp=None):
        # Construct and draw initial cubes
        max_s = 50.0
        ini_corridors = st_info.initial_corridors
        for path_index in range(len(ini_corridors.cubes_paths)):
            for cube_index in range(len(ini_corridors.cubes_paths[path_index].cubes)):
                cur_cube_msg = ini_corridors.cubes_paths[path_index].cubes[cube_index]
                cur_cube = Cube2D(cur_cube_msg.t_start, cur_cube_msg.t_end, cur_cube_msg.s_start, cur_cube_msg.s_end)
                max_s = max(max_s, cur_cube.s_end_)
                c_polygon = Polygon(cur_cube.vertex_)
                corre_ax.plot(*c_polygon.exterior.xy, c='k', ls=':') 

        # Construct and draw enhanced cubes
        enhanced_corridors = st_info.enhanced_corridors
        for path_index in range(len(enhanced_corridors.cubes_paths)):
            for cube_index in range(len(ini_corridors.cubes_paths[path_index].cubes)):
                cur_cube_msg = enhanced_corridors.cubes_paths[path_index].cubes[cube_index]
                cur_cube = Cube2D(cur_cube_msg.t_start, cur_cube_msg.t_end, cur_cube_msg.s_start, cur_cube_msg.s_end)
                c_polygon = Polygon(cur_cube.vertex_)
                corre_ax.plot(*c_polygon.exterior.xy, c='b') 

        # Draw st profile
        velocity_profile_info = st_info.velocity_profile_info
        corre_ax.plot(velocity_profile_info.t, velocity_profile_info.s, c='r', linewidth=1.5)

        # Draw occupied areas
        occupied_areas = st_info.occupied_areas
        for i in range(len(occupied_areas)):
            cur_parallelogram = occupied_areas[i]
            # # Draw with Polygon
            # vertice_0 = [cur_parallelogram.vertex[0].t, cur_parallelogram.vertex[0].s]
            # vertice_1 = [cur_parallelogram.vertex[1].t, cur_parallelogram.vertex[1].s]
            # vertice_2 = [cur_parallelogram.vertex[2].t, cur_parallelogram.vertex[2].s]
            # vertice_3 = [cur_parallelogram.vertex[3].t, cur_parallelogram.vertex[3].s]
            # vertex = np.array([vertice_0, vertice_1, vertice_2, vertice_3])
            # v_polygon = Polygon(vertex)
            # corre_ax.plot(*v_polygon.exterior.xy, c='grey')

            # # Draw with matploblib
            # corre_ax.fill([cur_parallelogram.vertex[0].t, cur_parallelogram.vertex[3].t, cur_parallelogram.vertex[2].t, cur_parallelogram.vertex[1].t], [cur_parallelogram.vertex[0].s, cur_parallelogram.vertex[3].s, cur_parallelogram.vertex[2].s, cur_parallelogram.vertex[1].s], color='grey')
            
            # Draw with multiple polygons
            for t_begin in np.arange(cur_parallelogram.vertex[0].t, cur_parallelogram.vertex[3].t, DRAW_DT):
                t_end = min(t_begin + DRAW_DT, cur_parallelogram.vertex[3].t)
                s_begin = cur_parallelogram.vertex[0].s + (t_begin / (cur_parallelogram.vertex[3].t - cur_parallelogram.vertex[0].t)) * (cur_parallelogram.vertex[3].s - cur_parallelogram.vertex[0].s)
                s_end = cur_parallelogram.vertex[1].s + (t_begin / (cur_parallelogram.vertex[2].t - cur_parallelogram.vertex[1].t)) * (cur_parallelogram.vertex[2].s - cur_parallelogram.vertex[1].s)
                corre_ax.fill([t_begin, t_end, t_end, t_begin], [s_begin, s_begin, s_end, s_end], color='grey')
                vertice_0 = [t_begin, s_begin]
                vertice_1 = [t_begin, s_end]
                vertice_2 = [t_end, s_end]
                vertice_3 = [t_end, s_begin]
                vertex = np.array([vertice_0, vertice_1, vertice_2, vertice_3])
                v_polygon = Polygon(vertex)
                corre_ax.plot(*v_polygon.exterior.xy, c='black')

        corre_ax.set_ylim(0.0, max_s)
        corre_ax.set_ylabel('s ($m$)')
        corre_ax.set_xlabel('t ($s$)')
        if last_update_timestamp:
            corre_ax.set_title('{} \n updated {:.3f} $s$ ago'.format(st_info.current_state_name, time.time() - last_update_timestamp))
        else:
            corre_ax.set_title(st_info.current_state_name)

    
    @staticmethod
    def supply_vt_fig(st_info, corre_ax):
        # Draw velocity profile
        velocity_profile_info = st_info.velocity_profile_info
        corre_ax.plot(velocity_profile_info.t, velocity_profile_info.v, c='r', linewidth=1.5)
        corre_ax.set_ylabel('v ($m/s$)')
        corre_ax.set_xlabel('t ($s$)')
    
    @staticmethod
    def supply_at_fig(st_info, corre_ax):
        # Draw velocity profile
        velocity_profile_info = st_info.velocity_profile_info
        corre_ax.plot(velocity_profile_info.t, velocity_profile_info.a, c='r', linewidth=1.5)
        corre_ax.set_ylabel('a ($m^{2}/s$)')
        corre_ax.set_xlabel('t ($s$)')

if __name__ == "__main__":
    rospy.init_node("st_graph_visualization_node")
    st_graph_re = StGraphRecorder()
    rospy.Subscriber("/velocity_planning/st_graph_interface", StGraph, st_graph_re.forward_st_graph_callback)
    rospy.Subscriber("/velocity_planning/st_graph_interface", StGraph, st_graph_re.turn_left_st_graph_callback)
    rospy.Subscriber("/velocity_planning/st_graph_interface", StGraph, st_graph_re.turn_right_st_graph_callback)


    plt.ion()
    fig = plt.figure(0, figsize=(10, 10))

    while not rospy.is_shutdown():
        # Wait information
        while not st_graph_re.forward_st_graph_info_ and not st_graph_re.left_st_graph_info_ and not st_graph_re.right_st_graph_info_:
            print('Wait information!!!')
            time.sleep(0.1)
        
        # Update figure
        ax_0 = fig.add_subplot(331)
        ax_1 = fig.add_subplot(332)
        ax_2 = fig.add_subplot(333)
        ax_3 = fig.add_subplot(334)
        ax_4 = fig.add_subplot(335)
        ax_5 = fig.add_subplot(336)
        ax_6 = fig.add_subplot(337)
        ax_7 = fig.add_subplot(338)
        ax_8 = fig.add_subplot(339)
        if st_graph_re.forward_st_graph_info_:
            StGraphVisualizer.supply_st_fig(st_graph_re.forward_st_graph_info_, ax_0, st_graph_re.forward_last_update_time_)
            StGraphVisualizer.supply_vt_fig(st_graph_re.forward_st_graph_info_, ax_3)
            StGraphVisualizer.supply_at_fig(st_graph_re.forward_st_graph_info_, ax_6)
        if st_graph_re.left_st_graph_info_:
            StGraphVisualizer.supply_st_fig(st_graph_re.left_st_graph_info_, ax_1, st_graph_re.turn_left_last_update_time_)
            StGraphVisualizer.supply_vt_fig(st_graph_re.left_st_graph_info_, ax_4)
            StGraphVisualizer.supply_at_fig(st_graph_re.left_st_graph_info_, ax_7)
        if st_graph_re.right_st_graph_info_:
            StGraphVisualizer.supply_st_fig(st_graph_re.right_st_graph_info_, ax_2, st_graph_re.turn_right_last_update_time_)
            StGraphVisualizer.supply_vt_fig(st_graph_re.right_st_graph_info_, ax_5)
            StGraphVisualizer.supply_at_fig(st_graph_re.right_st_graph_info_, ax_8)



        plt.pause(0.0001)
        plt.clf()



    rospy.spin()