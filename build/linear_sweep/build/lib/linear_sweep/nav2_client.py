import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile

import numpy as np


class NavigationClient(Node):
    def __init__(self):
        super().__init__("nav2_client")
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', QoSProfile(depth=10))
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSProfile(depth=10))
        self.map_data = None

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = yaw
        goal_pose.pose.orientation.w = 1.0  # Assuming a quaternion for simplicity

        goal_msg.pose = goal_pose

        self.goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.goal_future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        self.current_status = feedback_msg.feedback.status


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.current_status = GoalStatus.STATUS_REJECTED
            return

        self.get_logger().info('Goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)


    def result_callback(self, future):
        result = future.result().result
        self.current_status = future.result().status
        if result:
            self.get_logger().info('Goal reached')
        else:
            self.get_logger().info('Failed to reach goal')


    # def check_goal_status(self):
    #     if self.current_status == GoalStatus.STATUS_SUCCEEDED:
    #         return "finished"
    #     elif self.current_status in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_REJECTED]:
    #         return "not doable"
    #     elif self.current_status == GoalStatus.STATUS_UNKNOWN:
    #         return "unknown"
    #     else:
    #         return "in progress"


    def set_initial_pose(self, x, y, yaw):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = "map"
        initial_pose_msg.pose.pose.position.x = x
        initial_pose_msg.pose.pose.position.y = y
        initial_pose_msg.pose.pose.orientation.z = yaw
        initial_pose_msg.pose.pose.orientation.w = 1.0  # Assuming quaternion
        self.initial_pose_publisher.publish(initial_pose_msg)


    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map data received")


    def get_map(self):
        return self.map_data


    def display_map(self):
        if self.map_data is None:
            self.get_logger().info("No map data available")
            return

        # Convert the OccupancyGrid data to a numpy array
        map_array = np.array(self.map_data.data)
        width, height = self.map_data.info.width, self.map_data.info.height
        map_array = np.reshape(map_array, (height, width))

        # Normalize the array values to be in the range [0, 255]
        map_array = ((map_array + 100) * 255 / 200).astype(np.uint8)


# Example usage
def main(args=None):
    rclpy.init(args=args)
    navigation_client = NavigationClient()

    # Set initial pose
    # navigation_client.set_initial_pose(0.0, 0.0, 0.0)  # Modify as needed

    # Wait for map data
    while rclpy.ok():
        rclpy.spin_once(navigation_client)
        if navigation_client.get_map() is not None:
            break

    # Work with map data
    map_data = navigation_client.get_map()
    if map_data:
        print("Map received with resolution: ", map_data.info.resolution)

    navigation_client.send_goal(6.0, 6.0, 0.0)

    navigation_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
