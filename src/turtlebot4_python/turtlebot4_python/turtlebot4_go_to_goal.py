import rclpy
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
from irobot_create_msgs.action import NavigateToPosition
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
import tf_transformations
import numpy as np

class GoalSubscriberNode(Node):
    def __init__(self):
        super().__init__('goal_subscriber_node')
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Create subscriber to listen to robot_goal messages
        self.goal_subscriber = self.create_subscription(
            Point,
            '/robot_goal',
            self.goal_callback,
            10
        )
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.current_point = None
        self.action_client = ActionClient(self, NavigateToPosition, '/navigate_to_position')
        self.goal_in_progress = False  # Flag to track active goal

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.yaw) = tf_transformations.euler_from_quaternion(
        [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def goal_callback(self, msg):
        if not self.goal_in_progress:
            self.get_logger().info(f"Received goal: {msg.x}, {msg.y}")
            # Save the current goal
            self.current_point = [msg.x, msg.y]
            # Convert the Point message into NavigateToPosition goal and send it
            self.send_goal_to_robot(self.current_point)
        else:
            self.get_logger().info("Goal already in progress. Ignoring new goal.")

    def send_goal_to_robot(self, goal):
        # Create NavigateToPosition goal message
        goal_msg = NavigateToPosition.Goal()
        goal_msg.goal_pose.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        goal_msg.goal_pose.pose.position.x = goal[0]
        goal_msg.goal_pose.pose.position.y = goal[1]
        goal_msg.goal_pose.pose.position.z = 0.0  # Set to 0 for 2D navigation
        # Calculate orientation based on the current position and goal
        delta_x = goal[0] - self.robot_x
        delta_y = goal[1] - self.robot_y
        yaw = np.arctan2(delta_y, delta_x)  # Calculate yaw angle to the goal
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)

        # Set the orientation in quaternion
        goal_msg.goal_pose.pose.orientation.x = quaternion[0]
        goal_msg.goal_pose.pose.orientation.y = quaternion[1]
        goal_msg.goal_pose.pose.orientation.z = quaternion[2]
        goal_msg.goal_pose.pose.orientation.w = quaternion[3]
        goal_msg.achieve_goal_heading = False
        goal_msg.max_translation_speed = 0.3
        goal_msg.max_rotation_speed = 1.9

        # Check if the action client is ready
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        # Send goal asynchronously
        self.goal_in_progress = True
        self.get_logger().info(f"Sending goal to action server: {goal[0]}, {goal[1]}")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server.")
            self.goal_in_progress = False
            return

        self.get_logger().info("Goal accepted by action server. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully!")
        else:
            self.get_logger().error(f"Goal failed with status: {status}")

        # Reset the goal state
        self.goal_in_progress = False
        self.current_point = None


def main(args=None):
    rclpy.init(args=args)
    goal_subscriber_node = GoalSubscriberNode()

    try:
        rclpy.spin(goal_subscriber_node)
    except KeyboardInterrupt:
        pass

    goal_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
