#!/usr/bin/env python3 
import time
import json
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.parameter import ParameterValue
from std_msgs.msg import Header
import sys

class Pose3D:
    def __init__(self, x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1):
        self.pose_stamped = PoseStamped()
        self.pose_stamped.pose.position = Point()
        self.pose_stamped.pose.position.x = x
        self.pose_stamped.pose.position.y = y
        self.pose_stamped.pose.position.z = z
        self.pose_stamped.pose.orientation = Quaternion()
        self.pose_stamped.pose.orientation.x = qx
        self.pose_stamped.pose.orientation.y = qy
        self.pose_stamped.pose.orientation.z = qz
        self.pose_stamped.pose.orientation.w = qw
        self.pose_stamped.header = Header()
        self.pose_stamped.header.frame_id = 'map'

class RobotNavigation:

    def __init__(self):
        rclpy.init() 
        self.poses = self.load_poses()

    def load_poses(self):
        with open("/home/susan/BuddyDemoAGI/src/buddy_AGI/json/poses.json", "r") as file:
            data = json.load(file)

        # Convert the JSON data to a dictionary of Pose3D objects
        poses = {}
        for item in data:  # Note the change here
            position = item["position"]
            orientation = item["orientation"]
            pose = Pose3D(position["x"], position["y"], position["z"],
                        orientation["x"], orientation["y"], orientation["z"], orientation["w"])
            pose_name = item["pose_name"]
            poses[pose_name] = pose.pose_stamped
        return poses

    def move_and_wait(self, pose_name, wait_time):
        nav = BasicNavigator()
        nav.waitUntilNav2Active()  
        if pose_name in self.poses:
            target_pose = self.poses[pose_name]
            nav.goThroughPoses([target_pose])
            
            i = 0
            while not nav.isTaskComplete(): 
                i += 1
                feedback = nav.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated distance remaining to goal position: ' +
                        '{0:.3f}'.format(feedback.distance_remaining) +
                        '\nCurrent speed of the robot: ' +
                        '{0:.3f}'.format(feedback.number_of_poses_remaining))
                   
            print('Goal succeeded!')                     
            time.sleep(wait_time)  # wait for the specified amount of time before moving to the next pose
       
                           
    def continuous_navigation(self, list_pose_json_file):
        with open(list_pose_json_file, 'r') as file:
            list_pose_data = json.load(file) 
        
        repeat_time = int(list_pose_data["repeat_time"])
        wait_time = int(list_pose_data["wait_time"])
        list_poses = list_pose_data["list_pose"].split(", ")

        for _ in range(repeat_time):
            for pose_name in list_poses:
                self.move_and_wait(pose_name, wait_time)     

if __name__ == '__main__':
    # rclpy.init() 
    nav_test = RobotNavigation()
    # RobotNavigation().continuous_navigation("/home/susan/BuddyDemoAGI/src/buddy_AGI/json/pose_sequence.json")
    # RobotNavigation().continuous_navigation("/home/susan/BuddyDemoAGI/src/buddy_AGI/json/poseswir.json")
    nav_test.continuous_navigation("/home/susan/BuddyDemoAGI/src/buddy_AGI/json/list_pose.json")
