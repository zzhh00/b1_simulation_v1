import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
import json

class NavGoalFollower(Node):

    def __init__(self):
        super().__init__('nav_goal_follower')
        self.nav_goals = self.load_goals_from_json()
        self.goal_index = 0
        self.follow_path_client = self.create_client(FollowPath, '/follow_path')
        while not self.follow_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Follow Path service not available, waiting...')

    def load_goals_from_json(self):
        try:
            with open('/home/hz/hz/Work/Grabotics/B1-project/B1-Simulation/b1_simulation_v1/src/buddy_nav2/json/nav_goals.json', 'r') as json_file:
                goals = json.load(json_file)
            return goals
        except FileNotFoundError:
            self.get_logger().error('JSON file not found. Make sure to create nav_goals.json')
            return []

    def send_goal_to_follow_path(self, pose):
        goal_msg = FollowPath.Goal()
        goal_msg.poses.append(pose)
        self.follow_path_client.wait_for_service()
        future = self.follow_path_client.call_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

    def follow_navigation_goals(self):
        while rclpy.ok() and len(self.nav_goals) > 0:
            current_goal = self.nav_goals[self.goal_index]
            pose = PoseStamped()
            pose.pose.position.x = current_goal['position']['x']
            pose.pose.position.y = current_goal['position']['y']
            pose.pose.position.z = current_goal['position']['z']
            pose.pose.orientation.x = current_goal['orientation']['x']
            pose.pose.orientation.y = current_goal['orientation']['y']
            pose.pose.orientation.z = current_goal['orientation']['z']
            pose.pose.orientation.w = current_goal['orientation']['w']
            
            self.send_goal_to_follow_path(pose)
            self.goal_index = (self.goal_index + 1) % len(self.nav_goals)

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalFollower()
    node.follow_navigation_goals()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
