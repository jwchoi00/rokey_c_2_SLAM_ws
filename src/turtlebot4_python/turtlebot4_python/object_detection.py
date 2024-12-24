import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.node import Node
import numpy as np
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
class Search_Img(Node):
    def __init__(self):
        super().__init__('search_img')
        self.camera_offset = np.array([0.8, 0.05, 0.2])  # 카메라는 로봇 근처에 위치
        # Odometry 구독 (Best Effort로 설정)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile)
        #소화기, 사람 이미지 경로 설정
        ext_orig = '/home/psb/turtlebot3_ws/src/turtlebot3/turtlebot3_teleop/turtlebot3_teleop/script/ext_orig.png'#/home/psb/turtlebot3_ws/src/turtlebot3/turtlebot3_teleop/turtlebot3_teleop/script/ext_orig.png
        man_orig = '/home/psb/turtlebot3_ws/src/turtlebot3/turtlebot3_teleop/turtlebot3_teleop/script/man_orig.png'
        self.cv_ext_orig = cv2.imread(ext_orig)
        self.cv_man_orig = cv2.imread(man_orig)
        if self.cv_ext_orig is None:
            self.get_logger().error('Failed to load ext image.')
            return
        if self.cv_man_orig is None:
            self.get_logger().error('Failed to load man image.')
            return
        # 감지할 이미지가 실제 맵에서 회색으로 주어져 Grayscale로 변환
        self.ext_gray_image = cv2.cvtColor(self.cv_ext_orig, cv2.COLOR_BGR2GRAY)
        self.man_gray_image = cv2.cvtColor(self.cv_man_orig, cv2.COLOR_BGR2GRAY)
        #카메라 데이터 서브
        self.ImgSubscription = self.create_subscription(
            CompressedImage,#Image,
            '/oakd/rgb/preview/image_raw/compressed',# 카메라 이미지 토픽 이름 /camera/image_raw   /oakd/rgb/preview/image_raw
            self.imgSubscription_callback,
            10#qos_profile  # 큐 크기
        )
        self.bridge = CvBridge()
        #SIFT 변환 초기화. 이미지 확인 로직은 SIFT를 사용
        self.sift = cv2.SIFT_create()
        self.ext_keypoints, self.ext_descriptors = self.sift.detectAndCompute(self.ext_gray_image, None)
        self.man_keypoints, self.man_descriptors = self.sift.detectAndCompute(self.man_gray_image, None)
        self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        self.get_logger().info("SIFT Checker initialized.")
        # 캘리브레이션된 카메라 매트릭스
        self.camera_matrix = np.array([
            [201, 0, 125],  # fx, cx
            [0, 201, 130],  # fy, cy
            [0, 0, 1]
        ])
        self.distortion_coeffs = np.zeros((5, 1))  # 렌즈 왜곡 없음
        # 최신 이미지를 저장할 변수
        self.latest_image = None
        # tf2 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 타이머 생성 (1초 간격)
        self.timer = self.create_timer(1.0, self.searchImg)
    def odom_callback(self, msg):
        self.robot_position = msg.pose.pose.position
        self.robot_orientation = msg.pose.pose.orientation
        # 로봇의 방향 (쿼터니언 -> 롤, 피치, 요)
        orientation_q = [self.robot_orientation.x, self.robot_orientation.y, self.robot_orientation.z, self.robot_orientation.w]
        # 쿼터니언 -> 오일러 각 변환 (scipy 사용)
        r = R.from_quat(orientation_q)
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        # 로봇의 현재 위치 (x, y, z)
        self.robot_x = self.robot_position.x
        self.robot_y = self.robot_position.y
        self.robot_z = self.robot_position.z
        self.robot_yaw = yaw
    def imgSubscription_callback(self, msg):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)  # 압축된 데이터 -> NumPy 배열
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 압축 해제 (이미지 디코딩)
            #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image  # 최신 이미지 저장
        except Exception as e:
            self.get_logger().error(f"Error in processing image: {e}")
    def searchImg(self):
        if self.latest_image is None:
            self.get_logger().warn("No image received yet.")
            return
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            gray_cam_img = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
            kp, des = self.sift.detectAndCompute(gray_cam_img, None)
            if des is None:
                self.get_logger().warn("No descriptors found in current image.")
                return
            #소화기 탐지
            matches = self.matcher.match(self.ext_descriptors, des)
            matches = sorted(matches, key=lambda x: x.distance)
            # 일치율 계산
            ext_good_matches = [m for m in matches if m.distance < 100]
            num_ext_good_matches = len([m for m in matches if m.distance < 100])
            self.get_logger().info(f"ext Good Matches: {num_ext_good_matches}")
            # 시각화 (옵션)
            if num_ext_good_matches >= 10:  # 일치 기준 (조정 가능)
                self.get_logger().info("ext_Images Match!")
                object_points = [] # 3d 점
                image_points = [] #2d 점
                for match in ext_good_matches:
                    object_points.append(self.ext_keypoints[match.queryIdx].pt + (0,))  # (x, y, z)
                    image_points.append(kp[match.trainIdx].pt)  # (x, y)
                object_points = np.array(object_points, dtype=np.float32)
                image_points = np.array(image_points, dtype=np.float32)
                success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.distortion_coeffs)
                if success:
                    self.get_logger().info(f"Rotation Vector: {rvec.flatten()}")
                    self.get_logger().info(f"Translation Vector: {tvec.flatten()}")
                    # 카메라 좌표에서 월드 좌표로 변환
                    # 로봇의 위치 + 회전 변환을 통해 최종 월드 좌표 계산
                    rmat, _ = cv2.Rodrigues(rvec)
                    camera_world_position = np.dot(rmat, self.camera_offset) + np.array([self.robot_x, self.robot_y, self.robot_z])
                    self.get_logger().info(f"ramt {rmat}")
                    self.get_logger().info(f"Ext Position in Map: x={camera_world_position[0]:.2f}, y={camera_world_position[1]:.2f}, z={camera_world_position[2]:.2f}")
                    '''
                    cv2.imshow("SIFT Matching", self.latest_image)
                    key = cv2.waitKey(1)
                    if key == ord('q'):  # 'q' 키를 눌렀을 경우 이미지 창 닫기
                        cv2.destroyAllWindows()
                    '''
                else:
                    self.get_logger().error("PnP computation failed!")
            else:
                pass
            #사람 탐지
            matches = self.matcher.match(self.man_descriptors, des)
            matches = sorted(matches, key=lambda x: x.distance)
            # 일치율 계산
            man_good_matches = [m for m in matches if m.distance < 100]
            num_man_good_matches = len([m for m in matches if m.distance < 100])
            self.get_logger().info(f"Man Good Matches: {num_man_good_matches}")
            # 시각화 (옵션)
            if num_man_good_matches >= 10:  # 일치 기준 (조정 가능)
                self.get_logger().info("Man_Images Match!")
                object_points = [] # 3d 점
                image_points = [] #2d 점
                for match in man_good_matches:
                    object_points.append(self.man_keypoints[match.queryIdx].pt + (0,))  # (x, y, z)
                    image_points.append(kp[match.trainIdx].pt)  # (x, y)
                object_points = np.array(object_points, dtype=np.float32)
                image_points = np.array(image_points, dtype=np.float32)
                success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.distortion_coeffs)
                if success:
                    self.get_logger().info(f"Rotation Vector: {rvec.flatten()}")
                    self.get_logger().info(f"Translation Vector: {tvec.flatten()}")
                    # 카메라 좌표에서 월드 좌표로 변환
                    # 로봇의 위치 + 회전 변환을 통해 최종 월드 좌표 계산
                    rmat, _ = cv2.Rodrigues(rvec)
                    camera_world_position = np.dot(rmat, self.camera_offset) + np.array([self.robot_x, self.robot_y, self.robot_z])
                    self.get_logger().info(f"ramt {rmat}")
                    self.get_logger().info(f"Man Position in Map: x={camera_world_position[0]:.2f}, y={camera_world_position[1]:.2f}, z={camera_world_position[2]:.2f}")
                    '''
                    cv2.imshow("SIFT Matching", self.latest_image)
                    key = cv2.waitKey(1)
                    if key == ord('q'):  # 'q' 키를 눌렀을 경우 이미지 창 닫기
                        cv2.destroyAllWindows()
                    '''
                else:
                    self.get_logger().error("PnP computation failed!")
            else:
                pass
        except Exception as e:
            self.get_logger().error(f"Error in processing image: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = Search_Img()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Image Subscriber node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main()