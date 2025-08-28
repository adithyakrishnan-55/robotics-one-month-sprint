import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/src/my_ros2_pkg/install/my_ros2_pkg'
