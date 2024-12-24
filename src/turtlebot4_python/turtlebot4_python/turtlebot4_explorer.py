import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from heapq import heappop, heappush
import math

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
        # Robot state variables
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.origin = None
        self.resolution = 0.0
        self.grid = None

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
        self.get_logger().info(f"Robot float position: x={self.robot_x}, y={self.robot_y}")
        self.get_logger().info(f"Robot position: x={grid_x}, y={grid_y}")
        # After receiving the first valid odometry data and map, start exploration
        if self.grid is not None:
            self.explore()

    def find_closest_unknown(self):
        #self.get_logger().info("Looking for closest unknown cell.")
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
        #self.get_logger().info(f"Closest unknown cell: {closest_cell}")
        return tuple(closest_cell)


    def plan_path_to_target(self, start, goal):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
        open_list = []
        heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_list:
            _, current = heappop(open_list)

            if current == goal:
                break

            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if 0 <= neighbor[0] < self.grid.shape[0] and 0 <= neighbor[1] < self.grid.shape[1]:
                    if self.grid[neighbor[0], neighbor[1]] == -1 or self.grid[neighbor[0], neighbor[1]] >= 0:  # Free or unknown
                        new_cost = cost_so_far[current] + 1
                        if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                            cost_so_far[neighbor] = new_cost
                            priority = new_cost + np.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                            heappush(open_list, (priority, neighbor))
                            came_from[neighbor] = current

        # Reconstruct path
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                return None  # No path found
        path.reverse()
        return path

    def check_exploration_complete(self):
        unknown_cells = np.argwhere(self.grid == -1)
        result = unknown_cells.size == 0
        #self.get_logger().info(f"Exploration complete: {result}, Unknown cells left: {unknown_cells.size}")
        return result

    def explore(self):
        while not self.check_exploration_complete():
            #self.get_logger().info(f"Checking exploration status...")
            closest_unknown = self.find_closest_unknown()
            if closest_unknown is None:
                self.get_logger().info("No reachable unknown cells.")
                break

            # Get robot grid position
            robot_grid_x = int((self.robot_x - self.origin.position.x) / self.resolution)
            robot_grid_y = int((self.robot_y - self.origin.position.y) / self.resolution)

            path = self.plan_path_to_target((robot_grid_y, robot_grid_x), closest_unknown)
            #print(f'path_list ={path}')
            if path is None:
                self.get_logger().info("No path to the unknown cell.")
                break
            for target_x, target_y in path:
                trans_target_x = (target_x - self.robot_x) * self.resolution #grid좌표계를 거리 좌표계로 변환
                trans_target_y = (target_y - self.robot_y) * self.resolution #grid좌표계를 거리 좌표계로 변환
                #print(f'after target_x = {trans_target_x}')
                #print(f'after target_y = {trans_target_y}')
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
