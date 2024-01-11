import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math


class NavBridgeNode(Node):
    def __init__(self):
        super().__init__('grid_sweeper')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.last_target = (80, 80)
        # start_point = (80, 80)


    def get_position(self):
        return self.last_target


    def set_goal(self, x, y, theta):
        self.last_target = (x, y)


    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.action_client.wait_for_server()
        self.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))


def test(args=None):
    rclpy.init(args=args)
    action_client = NavBridgeNode()

    # Example goal
    action_client.send_goal(1.0, 1.0, 0.0)

    rclpy.spin(action_client)
    rclpy.shutdown()



if __name__ == "__main__":
    test()