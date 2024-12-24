import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from heapq import heappop, heappush
import math
from rclpy.timer import Timer
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class RobotVacuumMapper(Node):
    def __init__(self):
        super().__init__('robot_vacuum_mapper')
        
        # Publishers and subscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        # Action Client for Nav2
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Robot state variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.origin = None
        self.resolution = 0.0
        self.grid = None


    def send_nav_goal(self, target_x, target_y):
        # Wait for the action server to be available
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward, no rotation
        print('before sending go')
        # Send the goal to the Nav2 action server
        self.get_logger().info(f"Sending goal to Nav2: ({target_x}, {target_y})")
        print('before_wait_for_server')
        self.nav2_client.wait_for_server()
        print('wait_for_server')
        self.send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # def feedback_callback(self, feedback_msg):
    #     # Handle feedback (if needed)
    #     #self.get_logger().info(f"Feedback received: {feedback_msg}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        # Wait for the result asynchronously
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"Goal reached successfully: {result.result.status}")
        else:
            self.get_logger().warn(f"Goal failed: {result.result.status}")

    def map_callback(self, msg):
        # Update map information
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.grid = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        self.get_logger().info(f"Map updated: {self.width}x{self.height} with resolution {self.resolution} m/cell")

    def odom_callback(self, msg):
        if self.origin is None:
            self.get_logger().warn("Origin is not set, skipping position calculation.")
            return
        # Update robot position from odometry
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # Convert to grid coordinates
        grid_x = int((self.robot_x - self.origin.position.x) / self.resolution)
        grid_y = int((self.robot_y - self.origin.position.y) / self.resolution)
        # Call find_closest_unknown to check for nearest unexplored cell
        closest_unknown = self.find_closest_unknown()
        if closest_unknown:
            self.get_logger().info(f"Closest unknown: {closest_unknown}")
        # Log robot's position in grid
        self.get_logger().info(f"Robot position: x={grid_x}, y={grid_y}")
        # After receiving the first valid odometry data and map, start exploration
        if self.grid is not None:
            self.explore()

    def find_closest_unknown(self):
        self.get_logger().info("Looking for closest unknown cell.")
        if self.grid is None:
            self.get_logger().warn("No map data available.")
            return None

        # Find all unknown cells (-1)
        unknown_cells = np.argwhere(self.grid == -1)
        if unknown_cells.size == 0:
            self.get_logger().info("No unknown cells left.")
            return None  # No unknown cells left

        # Convert robot position to grid coordinates
        robot_grid_x = int((self.robot_x - self.origin.position.x) / self.resolution)
        robot_grid_y = int((self.robot_y - self.origin.position.y) / self.resolution)

        # Find the closest unknown cell
        closest_cell = min(
            unknown_cells,
            key=lambda cell: np.hypot(cell[0] - robot_grid_y, cell[1] - robot_grid_x)
        )
        self.get_logger().info(f"Closest unknown cell: {closest_cell}")
        return tuple(closest_cell)


    def follow_path(self, target_x, target_y):
        self.get_logger().info(f"Following path to ({target_x}, {target_y})")
        goal_reached = False
        while not goal_reached:
            self.send_nav_goal(target_x, target_y)


    def check_exploration_complete(self):
        unknown_cells = np.argwhere(self.grid == -1)
        result = unknown_cells.size == 0
        self.get_logger().info(f"Exploration complete: {result}, Unknown cells left: {unknown_cells.size}")
        return result



    def explore(self):
        closest_unknown = self.find_closest_unknown()
        if closest_unknown is None:
            self.get_logger().info("No reachable unknown cells.")
        # Get robot grid position
        robot_grid_x = int((self.robot_x - self.origin.position.x) / self.resolution)
        robot_grid_y = int((self.robot_y - self.origin.position.y) / self.resolution)
        path = closest_unknown
        print(path)
        # Follow the path, passing each target coordinate
        #self.follow_path(target_x, target_y)  # Reverse x and y because they are (y, x)
        self.get_logger().info("Exploration complete!")




def main(args=None):
    rclpy.init(args=args)
    node = RobotVacuumMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
