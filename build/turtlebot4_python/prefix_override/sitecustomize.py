import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/g1/rokey_c_2_SLAM_ws/install/turtlebot4_python'
