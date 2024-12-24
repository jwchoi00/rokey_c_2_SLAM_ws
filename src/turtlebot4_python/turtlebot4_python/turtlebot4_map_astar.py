import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from queue import PriorityQueue
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.ndimage import binary_dilation

def inflate_map(map_data, inflation_radius):
    obstacles = (map_data == 100)
    structure = np.ones((2*inflation_radius + 1, 2* inflation_radius + 1))
    inflated_obstacles = binary_dilation(obstacles, structure=structure)
    inflated_map = np.where(inflated_obstacles, 100, map_data)
    return inflated_map

def a_star(map_data, start, goal):
    height, width = map_data.shape
    open_list = PriorityQueue()
    open_list.put((0, start))  # (priority, current node)
    came_from = {}
    g_score = {start: 0}

    def heuristic(a, b):
        # Manhattan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    f_score = {start: heuristic(start, goal)}

    def neighbors(node):
        # Return valid neighboring cells
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        result = []
        for dx, dy in directions:
            x, y = node[0] + dx, node[1] + dy
            if 0 <= x < width and 0 <= y < height and map_data[y, x] == 0:  # Free cell
                result.append((x, y))
        return result

    while not open_list.empty():
        _, current = open_list.get()
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        for neighbor in neighbors(current):
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                open_list.put((f_score[neighbor], neighbor))
    return None  # No path found


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)

        # Publisher
        self.path_publisher = self.create_publisher(Path, '/path', 10)

        # Attributes
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.robot_position = None
        self.timer = self.create_timer(1.0, self.plan_path)

    def map_callback(self, msg):
        # Convert OccupancyGrid data to numpy array
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        raw_map_data = np.array(msg.data).reshape((height, width))
        inflation_radius = 1
        self.map_data = inflate_map(raw_map_data, inflation_radius)
        self.get_logger().info(f"Map received: {width}x{height}, resolution: {self.resolution}m")

    def odom_callback(self, msg):
        if self.origin is None:
            self.get_logger().warn("Origin is not set, skipping position calculation.")
            return
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.get_logger().info(f"Robot odometry: x={self.robot_x}, y={self.robot_y}")

        # Convert to map cell coordinates
        map_x = int((self.robot_x - self.origin.position.x) / self.resolution)
        map_y = int((self.robot_y - self.origin.position.y) / self.resolution)
        self.robot_position = (map_x, map_y)
        self.get_logger().info(f"Robot map position: x={map_x}, y={map_y}")


    def plan_path(self):
        # Check if map and robot position are ready

        if self.map_data is None:
            self.get_logger().info("Waiting for map...")
            return
        if self.robot_position is None:
            self.get_logger().info("Waiting for robot position...")
            return

        # Define start and goal (map cell coordinates)
        start = self.robot_position
        goal = (start[0] - 20, start[1] - 20)  # Example: replace with the desired goal coordinates

        # Compute the path
        path = a_star(self.map_data, start, goal)
        if path:
            self.get_logger().info(f"Path found: {path}")
            self.publish_path(path)
        else:
            self.get_logger().warn("No path found.")

    def publish_path(self, path):
        # Publish the computed path as a Path message
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x * self.resolution + self.origin.position.x
            pose.pose.position.y = y * self.resolution + self.origin.position.y
            path_msg.poses.append(pose)
        self.path_publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
