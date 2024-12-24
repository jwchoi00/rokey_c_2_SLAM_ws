import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

# class WallFollower(Node):
#     def __init__(self):
#         super().__init__('wall_follower')
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

#     def map_callback(self, msg):
#         # Extract map dimensions
#         width = msg.info.width
#         height = msg.info.height
#         resolution = msg.info.resolution
#         origin = msg.info.origin

#         # Reshape flat data into a 2D grid
#         grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

#         # Log map info
#         self.get_logger().info(f"Map received: {width}x{height} with resolution {resolution} m/cell")
#         for w in range(width):
#         # Print specific rows
#             row = grid[w,:]  # First row
#         # middle_row = grid[height // 2, :]  # Middle row
#         # last_row = grid[-1, :]  # Last row

#             self.get_logger().info(f"{w}row: {row}")
            
#         # self.get_logger().info(f"Middle row: {middle_row}")
#         # self.get_logger().info(f"Last row: {last_row}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = WallFollower()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy


class WallFollower(Node):
    def __init__(self):
        super().__init__('grid')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Define QoS profile for Odometry subscriber
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  # Adjust to match the publisher's QoS

        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.origin = None
        self.resolution = 0.0

    def map_callback(self, msg):
        # Extract map dimensions and grid
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin  # Store the origin from the map message

        grid = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        for w in range(self.width):
            print(grid[w,:])
        # Log map info
        self.get_logger().info(f"Map received: {self.width}x{self.height} with resolution {self.resolution} m/cell")

    def odom_callback(self, msg):
        if self.origin is None:
            self.get_logger().warn("Origin is not set, skipping position calculation.")
            return

        # Extract robot's position from odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Convert to grid coordinates
        grid_x = int((self.robot_x - self.origin.position.x) / self.resolution)
        grid_y = int((self.robot_y - self.origin.position.y) / self.resolution)

        # Log robot's position in grid
        self.get_logger().info(f"Robot position: x={grid_x}, y={grid_y}")

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



