import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import numpy as np

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')

        # Map 구독
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        # Marker 퍼블리셔 생성
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)

    def map_callback(self, msg):
        # OccupancyGrid 데이터 확인
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        # 1D 맵 데이터를 2D numpy 배열로 변환
        map_data = np.array(msg.data).reshape((height, width))
        self.get_logger().info(f"Map received: {width}x{height}, resolution: {resolution}m")

        # 특정 좌표를 선택
        x, y = 10, 10  # 특정 위치 (cell 단위)
        if 0 <= x < width and 0 <= y < height:
            cell_value = map_data[y, x]
            self.get_logger().info(f"Value at ({x}, {y}): {cell_value}")

            # RViz2에 좌표 표시
            self.publish_marker(x, y, resolution, origin)

    def publish_marker(self, x, y, resolution, origin):
        # 셀 좌표를 실제 좌표로 변환
        world_x = origin.x + x * resolution
        world_y = origin.y + y * resolution

        # Marker 메시지 생성
        marker = Marker()
        marker.header.frame_id = "map"  # RViz2에서 표시할 좌표계
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "map_coordinates"
        marker.id = 0
        marker.type = Marker.SPHERE  # 구 형태로 표시
        marker.action = Marker.ADD
        marker.pose.position.x = world_x
        marker.pose.position.y = world_y
        marker.pose.position.z = 0.0  # z 좌표 (2D 지도에서는 일반적으로 0)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # 구의 크기
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # 투명도 (1.0 = 불투명)
        marker.color.r = 1.0  # 빨간색
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Marker 퍼블리싱
        self.marker_publisher.publish(marker)
        self.get_logger().info(f"Marker published at ({world_x}, {world_y})")

def main(args=None):
    rclpy.init(args=args)
    node = MapSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
