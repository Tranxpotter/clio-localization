import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/iwintern/guide_robot_ws/src/guide_robot_localization/install/guide_robot_localization'
