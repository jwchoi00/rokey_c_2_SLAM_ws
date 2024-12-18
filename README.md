# rokey_c_2_SLAM_ws
이 프로젝트는 Turtlebot4를 활용하여 자율 탐색 목표 선정과 이동, object detection과 mapping을 하는 과제이다.
담당 부분
- turtlebot explorer
#Pseudo code
1. 초기화 및 설정
   ROS 2 노드 PathPlanner를 초기화.
   ROS 메시지와 토픽을 설정:
      /map 토픽을 구독하여 지도 데이터 수신.
      /odom 토픽을 구독하여 로봇의 현재 위치 추적.
      /path 토픽에 경로 데이터를 발행.
      /cmd_vel 토픽에 로봇 이동 명령을 발행.
   타이머를 사용해 반복 작업 수행:
      경로 계획(plan_path)을 0.5초 간격으로 실행.
      목표 지점까지 이동(move_to_goal) 명령을 0.05초 간격으로 발행.
2. 지도 데이터 처리
   지도 데이터(OccupancyGrid)를 수신한 후 numpy 배열로 변환.
   장애물 근처에 여유 공간을 두기 위해 inflate_map 함수로 지도 확장.
      장애물 반경에 대해 설정된 거리만큼 영역을 확장.
3. 로봇 위치 추적
   /odom 토픽에서 로봇의 위치 및 방향(roll, pitch, yaw)을 가져옴.
   로봇의 실제 위치를 지도 좌표계로 변환.
4. A* 경로 탐색 알고리즘
   A 알고리즘* 구현:
      **시작점(start)**에서 **목표점(goal)**까지 최적 경로 탐색.
      각 셀에서 이동 가능한 인접 셀(상하좌우 및 대각선)을 확인.
      휴리스틱 함수(유클리드 거리)를 사용하여 경로 비용 평가.
      최적 경로를 발견하면 반환.
   경로가 없을 경우 None 반환.
5. 프론티어 기반 목표 설정
   find_frontier_goal 함수로 목표 지점 설정:
      지도의 이동 가능한 셀 중 주변에 알려지지 않은(-1) 영역이 있는 셀을 프론티어로 식별.
      로봇과 가장 가까운 프론티어 셀을 목표로 선택.
6. 경로 계획 및 발행
   현재 지도 및 로봇 위치 정보를 바탕으로 경로를 계획.
   목표 프론티어 셀이 있을 경우 A* 알고리즘으로 경로 탐색.
   경로를 찾으면 ROS Path 메시지로 경로 발행.
      각 경로 점을 지도 좌표에서 실제 좌표로 변환.
7. 목표 지점으로 이동
   로봇이 목표 지점에 가까워지면 이동을 멈춤.
      목표 도달 조건: 거리 < threshold (예: 1m).
   목표까지의 방향과 거리 계산.
   로봇 이동 명령(Twist 메시지) 생성:
      선형 속도(linear.x): 목표 거리와 비례하여 설정.
      각속도(angular.z): 목표 방향과 현재 방향의 차이를 기반으로 설정.
8. 방향 제어
   로봇이 목표 방향으로 회전하도록 제어:
      목표 각도와 현재 각도의 차이를 계산.
      비례 제어기를 사용해 각속도 조정.
   최대 회전 속도를 초과하지 않도록 제한.
9. 주요 작업 반복
   경로 계획과 이동 명령을 타이머를 통해 반복적으로 수행.
   목표를 계속해서 업데이트하며 로봇을 이동시킴.
10. 종료
   ROS 2 노드를 종료하고 관련 리소스를 정리.

#문제점
문제1. 로봇의 사이즈를 고려하지 않고 A_star 경로 탐색을 사용하여 로봇이 벽이 부딪치는 현상이 발생
-> inflatation_radius를 주어 경로 생성시 로봇이 벽에 충돌하지 않도록 경로를 생성

![image](https://github.com/user-attachments/assets/1a6c027c-8c9b-4835-b7f6-ac3a3dca4384)

![image](https://github.com/user-attachments/assets/a213ad25-6e3e-400e-b43d-6682b4abb331)

문제2. inflation_radius를 너무 크게 줄 때 좁은 벽에서 경로 생성이 안되 로봇이 경로를 잃어버리는 현상이 발생
코드 개선 -> 중복되지 않는 좌표를 지속적으로 저장하고, 로봇이 지나간 좌표는 제거하는 방식으로 개선 시도(미구현)

![image](https://github.com/user-attachments/assets/ac7a9719-877e-46f9-afae-0c3c4c96fade)

문제 3. 현재 좌표와 이동 목표 좌표를 직선 거리와 theta를 계산하여 로봇의 이동 거리와 진행 방향을 설정
->로봇 진행 방향 속도가 생각보다 빨라서 회전 반경이 충분하지 않아 벽에 충돌
개선 -> 로봇의 진행 방향 속도를 강제로 낮추어 회전 반경을 축소시켜 벽과의 충돌이 감소

#어려웠던 점
/map에서 내 로봇의 좌표 찾기
이유 -> 내 좌표로 부터 목표 지점까지의 경로 생성을 하기 위하여
 사용한 계산 법: (/odom으로 받아온 로봇 좌표(msg.pose.pose.position.x)-지도 하단의 픽셀 좌표(msg.info.origin))/해상도(msg.info.resolution)
2. /찾은 목표 지점을 실제 좌표로 변환
이유 -> 기존에 /map_grid 좌표 기준으로 path를 생성하니 로봇이 이상하게 움직임(당연하다 rviz에서 보여주는 건 실제 좌표 기준이니까)
사용한 계산법: (map_grid좌표 * 해상도) + 지도 하단의 픽셀 좌표
3. 목표 지점 선정 frontier 탐색사용
이유 -> 로봇의 이동 목표는 미탐지 영역이다. 아는 영역을 기준으로 모르는 영역을 탐색해야 좀 더 빠르게 탐색할 수 있다고 생각 + 로봇으로 부터 가장 가까운 frontier point를 목표 지점으로 선정
사용 방법 :
1. /map에서 모든 0인 영역인 좌표를 받아온다.
2. 좌표를 기준으로 상하 좌우를 비교하여 가장 많은 미탐색 영역이 있는 좌표를 frontier 후보군으로 선정
3. frontier 후보군 중 로봇과 가장 가까운 좌표를 선정
문제 발생 -> 문제 1로 연결됨
10:08
중요하다고 생각했던 점: 로봇의 좌표 변환(실제 좌표 to map_grid, map_grid to 실제 좌표)
이유-> 로봇의 좌표변환이 유연하게 이루어 져야 내가 세운 기준으로 로봇을 마음대로 이동시킬 수 있기 때문에
