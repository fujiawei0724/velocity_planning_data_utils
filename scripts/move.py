#!/usr/bin/env python

import rospy
import random
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose,Point,Quaternion,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import time
class random_move:

    pose_list = []
    temp_pose_list = []
    success_move = 0
    
    
    def __init__(self):
        rospy.init_node('movebase_client_py')
        self.initial_pose_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.get_pose()
        self.main_loop()

    def odom_callback(self, data):#fps:30hz
        rospy.loginfo("odom pose: x:%0.6f , y:%0.6f",data.pose.pose.position.x,data.pose.pose.position.y)
        self.real_pose = data.pose

    def pub_initial_pose(self,pose):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.pose = pose
        self.initial_pose_pub.publish(initial_pose)

    def movebase_client(self):


        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.temp_pose_list[random.randint(0,len(self.temp_pose_list)-1)]
        # goal.target_pose.pose = self.temp_pose_list.pop(random.randint(0,len(self.temp_pose_list)-1))
        # goal.target_pose.pose.position.x = 0.5
        # goal.target_pose.pose.position.y = 0.5
        # goal.target_pose.pose.orientation.z = 0.702
        # goal.target_pose.pose.orientation.w = 0.71

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result(),self.client.get_state()

    def add_pose(self,pose):
        self.pose_list.append(pose)

    def get_pose(self):
        points_seq = rospy.get_param('movebase_client_py/p_seq')
        qua_seq = rospy.get_param('movebase_client_py/qua_seq')
        
        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        quas = [qua_seq[i:i+4] for i in range(0, len(qua_seq), 4)]
        for i in range(len(points)):
            self.pose_list.append(Pose(Point(*points[i]),Quaternion(*quas[i])))

        self.temp_pose_list = self.pose_list

    def main_loop(self):
        #start
        self.pub_initial_pose(self.real_pose)
        start_time = time.time()
        #todo: start to record plot data

        while (self.success_move <= 2)and(not rospy.is_shutdown()):
            result,state = self.movebase_client()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("the result of sever is:success")
                self.success_move += 1
            else:
                rospy.loginfo("move is not success, the state is "+ str(state))

if __name__ == '__main__':
    try:
        
        mv = random_move()
        # mv.add_pose(Pose(Point(-1,-2,0),Quaternion(0,0,0,1)))
        # mv.get_pose()
        # mv.main_loop()
        # result,state = mv.movebase_client()
        # if state == GoalStatus.SUCCEEDED:
        #     rospy.loginfo("the result of sever is:success")
        # else:
        #     rospy.loginfo("move is not success, the state is "+ str(state))
        # if result:
        #     rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")