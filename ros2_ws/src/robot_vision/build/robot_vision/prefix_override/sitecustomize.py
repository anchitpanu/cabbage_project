import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jorjeen/plant/cabbage_project/ros2_ws/src/robot_vision/install/robot_vision'
