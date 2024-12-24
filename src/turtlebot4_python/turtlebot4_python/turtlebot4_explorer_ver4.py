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
        self.goal_sent = False #골을 여러번 보내지 않기 위해
        self.goal_reached = False  # goal 도달 여부 플래그
        
    def map_callback(self, msg): #map에 대한 정보를 가져와
        # Update map information
        self.width = msg.info.width #가로 길이 현재 rviz의 좌표계는
        self.height = msg.info.height #세로 길이
        self.resolution = msg.info.resolution #축적 비율
        self.origin = msg.info.origin #처음 위치
        self.grid = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width)) #map에 대한 좌표 정보 리스트 형태
        self.get_logger().info(f"Map updated: {self.width}x{self.height} with resolution {self.resolution} m/cell") #현재 출력

    def odom_callback(self, msg): #내 로봇 위치 가져오기
        if self.origin is None: # 만약 map의 기준점이 없다면 취소
            self.get_logger().warn("Origin is not set, skipping position calculation.")
            return

        self.robot_x = msg.pose.pose.position.x #현재 로봇 x좌표 map to base
        self.robot_y = msg.pose.pose.position.y #현재 로봇 y좌표 map to base
        
        self.get_logger().info(f"Robot float position: x={self.robot_x}, y={self.robot_y}")
        # self.get_logger().info(f"self.origin.position: x={self.origin.position.x}, y={self.origin.position.y}")
        # # Convert to grid coordinates
        # grid_origin_x = self.origin.position.x / self.resolution
        # grid_origin_y = self.origin.position.y / self.resolution
        # grid_x = (self.robot_x - self.origin.position.x) / self.resolution #현재 로봇 float길이를 좌표로 변환
        # grid_y = (self.robot_y - self.origin.position.y) / self.resolution #현재 로봇 float길이를 좌표로 변환
        # # Call find_closest_unknown to check for nearest unexplored cell
        # self.get_logger().info(f"origin position: x={grid_origin_x}, y={grid_origin_y}")
        # self.get_logger().info(f"Robot position: x={grid_x}, y={grid_y}")
        closest_unknown = self.find_closest_unknown() #가져오 map 리스트에서 가장 가까운 미탐지 영역 찾기, 이때 좌표는 grid 기준 send_goal의 좌표로는 쓸 수 없다.
        if closest_unknown: #만약 좌표가 있다면
            self.get_logger().info(f"Closest unknown: {closest_unknown}") #그 좌표를 밷어라

        # if self.grid is not None: #만약 map에 대한 좌표 정보 리스트있다면
        #     self.goal_sent = True #goal 보내기 승인
            self.check_goal_reached(closest_unknown[1],closest_unknown[0]) #가장 가까운 좌표로 send_goal

    def check_goal_reached(self, target_x, target_y):
        if not self.goal_sent:  # goal이 전송되지 않았다면
            return
        
        self.get_logger().info(f"Checking if goal reached: target_x={target_x}, target_y={target_y}")
        if abs(self.robot_x - target_x) < self.resolution and abs(self.robot_y - target_y) < self.resolution:
            self.goal_reached = True
            self.get_logger().info(f"Goal reached: ({target_x}, {target_y})")
    # def check_goal_reached(self,target_x, target_y): #send_goal이 True이면 send_goal을 보내는 과정으로 이동 아니라면 안보냄
    #     if self.goal_sent: #만약 goal보내기 승인이 되었다면
    #         print(f'target_x={target_x}') #closest_unknown[1]
    #         print(f'target_y={target_y}') #closest_unknown[0]
    #         self.follow_path(target_x, target_y) #좌표로 이동하라

    def find_closest_unknown(self): #모르는 좌표 찾기
        self.get_logger().info("Looking for closest unknown cell.")
        if self.grid is None: #만약 제공받은 map에대한 정보가 없다면 코드 정지
            self.get_logger().warn("No map data available.")
            return None
        # Find all unknown cells (-1)
        unknown_cells = np.argwhere(self.grid == -1) #리스트에서 -1의 값을 값는 좌표 개수 탐색
        if unknown_cells.size == 0: #만약 좌표가 하나도 없다면 미탐색 좌표 없음
            self.get_logger().info("No unknown cells left.")
            return None  # No unknown cells left
        # Convert robot position to grid coordinates
        robot_grid_x = int((self.robot_x - self.origin.position.x) / self.resolution) #현재 로봇 좌표를 grid좌표계로 변환
        robot_grid_y = int((self.robot_y - self.origin.position.y) / self.resolution) #현재 로봇 좌표를 grid좌표계로 변환
        # Find the closest unknown cell
        closest_cell = min( #가장 작은 가까운 좌표 찾기
            unknown_cells,
            key=lambda cell: np.hypot(cell[0] - robot_grid_y, cell[1] - robot_grid_x)
        )
        self.get_logger().info(f"Closest unknown cell: {closest_cell}")
        return tuple(closest_cell)

    def follow_path(self, target_x, target_y):
        self.get_logger().info(f"Following path to ({target_x}, {target_y})")
        real_target_x = (target_x * self.resolution) + self.origin.position.x
        real_target_y = (target_y * self.resolution) + self.origin.position.y
        self.get_logger().info(f"Converted target to real coordinates: ({real_target_x}, {real_target_y})")

        self.send_nav_goal(real_target_x, real_target_y)

        # 비동기적으로 목표를 보낸 후 피드백을 처리하고 목표가 완료되었는지 확인
        while not self.goal_reached:
            rclpy.spin_once(self)  # 이벤트 루프 돌리기
            if self.goal_reached:
                self.get_logger().info(f"Goal reached: ({target_x}, {target_y})")

    # def follow_path(self, target_x, target_y): #받은 좌표로 이동하기
    #     self.get_logger().info(f"Following path to ({target_x}, {target_y})")
    #     # Send the navigation goal synchronously and wait for the result
    #     goal_reached = False #goal에 도착을 못하였다면 while문 들어가기
    #     while not goal_reached: #도착을 못했다면
    #         real_target_x = (target_x * self.resolution) + self.origin.position.x
    #         real_target_y = (target_y * self.resolution) + self.origin.position.y
    #         self.get_logger().info(f"Converted target to real coordinates: ({real_target_x}, {real_target_y})")

    #         print(f'after target_x = {real_target_x}')
    #         print(f'after target_y = {real_target_y}')
    #         self.get_logger().info("Sending navigation goal...")
    #         self.send_nav_goal(real_target_x, real_target_y)
    #         self.get_logger().info("Goal sent. Waiting for feedback...")
    #         #print('여기있어요')
    #         # Wait for the goal to be reached
    #         feedback_received = False #피드백 받기 대기
    #         while not feedback_received: #피드백이 없다면
    #             rclpy.spin_once(self) #코드 한번 실행 -> odom과 map callback 실행시켜 최신화가 목적
    #             print(f'self.robot_x = {self.robot_x}')
    #             print(f'self.robot_y = {self.robot_y}')
    #             # 목표에 도달했는지 확인하세요(여기에서 피드백 또는 odom을 확인할 수 있습니다)
    #             # 로봇이 목표에 도달했는지 확인하기 위해 논리를 추가합니다
    #             # 이것은 로봇의 위치나 액션 서버의 피드백을 기반으로 할 수 있습니다
    #             if abs(self.robot_x - real_target_x) < self.resolution and abs(self.robot_y - real_target_y) < self.resolution:
    #                 feedback_received = True #while문 탈출 목적
    #                 goal_reached = True #while문 탈출 목적
    #                 self.get_logger().info(f"Goal reached: ({target_x}, {target_y})")

    def send_nav_goal(self, target_x, target_y): #goal로 가라
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
        self.goal_sent = True
        # Send the goal to the Nav2 action server
        self.get_logger().info(f"Sending goal to Nav2: ({target_x}, {target_y})")
        self.send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Handle feedback (if needed)
        self.get_logger().info(f"Feedback received: {feedback_msg}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.goal_sent = False
            return
        self.get_logger().info('Goal accepted :)')
        # Wait for the result asynchronously
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 6:  # ABORTED
            self.get_logger().error('Goal was aborted by the server.')
            # Handle the failure, e.g., retry or log the error
        elif result.status == 3:  # SUCCEEDED
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().warn(f'Goal finished with status: {result.status}')


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
