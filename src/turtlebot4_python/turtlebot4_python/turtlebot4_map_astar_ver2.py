import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from queue import PriorityQueue
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.ndimage import binary_dilation
import tf_transformations
from geometry_msgs.msg import Twist

def inflate_map(map_data, inflation_radius):
    obstacles = (map_data == 100)
    structure = np.ones((2 * inflation_radius + 1, 2 * inflation_radius + 1))
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
        # 유클리드 거리 계산
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    f_score = {start: heuristic(start, goal)}

    def neighbors(node):
        # 8방향 (상하좌우 + 대각선) 이동
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 상하좌우
            (-1, -1), (-1, 1), (1, -1), (1, 1)  # 대각선
        ]
        result = []
        for dx, dy in directions:
            x, y = node[0] + dx, node[1] + dy
            if 0 <= x < width and 0 <= y < height and map_data[y, x] == 0:  # 이동 가능한 공간
                result.append((x, y))
        return result

    while not open_list.empty():
        _, current = open_list.get()
        if current == goal:
            # 경로 재구성
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        for neighbor in neighbors(current):
            # 가중치 계산: 대각선 이동인지 확인
            move_cost = 1.414 if abs(neighbor[0] - current[0]) == 1 and abs(neighbor[1] - current[1]) == 1 else 1
            tentative_g_score = g_score[current] + move_cost
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                open_list.put((f_score[neighbor], neighbor))
    return None  # 경로를 찾지 못한 경우


def find_frontier_goal(map_data, current_position):
    height, width = map_data.shape
    frontier_cells = []

    for y in range(height):
        for x in range(width):
            if map_data[y, x] == 0:  # 이동 가능한 공간
                # 주변에 알려지지 않은 영역(-1)이 있는지 확인
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height and map_data[ny, nx] == -1:
                        frontier_cells.append((x, y))
                        break

    # 가장 가까운 프론티어 선택
    if frontier_cells:
        return min(frontier_cells, key=lambda cell: ((cell[0] - current_position[0]) ** 2 +
                                                     (cell[1] - current_position[1]) ** 2) ** 0.5)

    return None  # 프론티어가 없는 경우


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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Attributes
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.robot_position = None
        self.target_position = None
        self.path = None
        self.previous_path = None
        #self.timer = self.create_timer(0.1, self.plan_and_move)  # 경로 계획과 이동을 동시에 처리하는 타이머
        # Timers
        self.plan_timer = self.create_timer(0.5, self.plan_path)  # Path planning every 0.5 seconds
        self.plan_timer = self.create_timer(0.5, self.move_to_goal)

    def map_callback(self, msg):
        # OccupancyGrid 데이터를 numpy 배열로 변환
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        raw_map_data = np.array(msg.data).reshape((height, width))
        inflation_radius = 13#장애물과의 거리 현재 (19,19)pixel만큼 떨어저서 이동
        self.map_data = inflate_map(raw_map_data, inflation_radius)
        if self.marked_map_data == None:
            self.marked_map_data = raw_map_data
            print(self.marked_map_data)
        #self.get_logger().info(f"Map received: {width}x{height}, resolution: {self.resolution}m")

    def odom_callback(self, msg):
        if self.origin is None:
            self.get_logger().warn("Origin is not set, skipping position calculation.")
            return
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
        [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        #self.get_logger().info(f"Robot odometry: x={self.robot_x}, y={self.robot_y}")

        # 지도 좌표계로 변환
        map_x = int((self.robot_x - self.origin.position.x) / self.resolution)
        map_y = int((self.robot_y - self.origin.position.y) / self.resolution)
        self.robot_position = (map_x, map_y, yaw)
        self.get_logger().info(f"Robot map position: x={map_x}, y={map_y}, yaw={yaw}")

    def move_to_goal(self):
        if self.target_position is None or self.robot_position is None:
            return

        goal_x, goal_y = self.target_position
        robot_x, robot_y, _ = self.robot_position

        # Calculate the distance to the goal
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = np.sqrt(dx**2 + dy**2)
        print(f"My current target = {goal_x}, {goal_y}")
        # Define a threshold to stop
        threshold = 1.0  # meters
        print(f"distance = {distance}")
        # If we're close enough to the goal, stop
        if distance < threshold:
            self.get_logger().info("Goal reached!")
            self.cmd_vel_publisher.publish(Twist())  # Stop the robot
            return

        # Otherwise, move towards the goal
        cmd_vel = Twist()
        # Move forward (linear velocity)
        cmd_vel.linear.x = min(0.1, distance)  # Max linear speed is 0.2 m/s
        # Rotate towards the goal (angular velocity)
        angle_to_goal = np.arctan2(dy, dx)
        current_angle = self.get_robot_orientation()

        # Control the robot's rotation (turn towards the goal)
        angle_diff = angle_to_goal - current_angle
        cmd_vel.angular.z = self.control_rotation(angle_diff)

        # Send velocity command to the robot
        self.cmd_vel_publisher.publish(cmd_vel)

    def control_rotation(self, angle_diff):
        # Control the rotation to align with the goal
        max_rot_speed = 2.0  # Max angular velocity in radians/s
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        elif angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        # Proportional controller for rotation
        angular_speed = max(-max_rot_speed, min(max_rot_speed, 0.5 * angle_diff))  # Gain = 0.5
        return angular_speed

    def get_robot_orientation(self):
        # This method can be used to fetch the robot's current orientation (yaw) from Odometry
        # Assuming the robot's orientation is stored in the Odometry message
        if self.robot_position:
            return self.robot_position[2]  # Assuming the orientation is part of robot_position
        return 0.0  # Default value if orientation is unknown

    def plan_path(self):
        # 맵과 로봇 위치가 준비되었는지 확인
        if self.map_data is None:
            self.get_logger().info("Waiting for map...")
            return
        if self.robot_position is None:
            self.get_logger().info("Waiting for robot position...")
            return

        # 시작점 및 목표점 설정
        start = self.robot_position
        frontier_goal = find_frontier_goal(self.map_data, (start[0], start[1]))

        if frontier_goal:
            self.get_logger().info(f"Planning path to frontier: {frontier_goal}")
            self.target_position = frontier_goal
            path = a_star(self.map_data, (start[0], start[1]), frontier_goal)
            
            if path:
                self.get_logger().info(f"Path found: {path}")
                self.publish_path(path)
                self.target_position = path[1]
            else:
                self.get_logger().warn("No path found.")
        else:
            self.get_logger().warn("No frontier goal found.")

    def publish_path(self, path):
        # Path 메시지로 경로 발행
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
