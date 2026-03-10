import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jorjeen/plant/cabbage_project/install/robot_dashboard'
