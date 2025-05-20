import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rokey-jw/rokeyracing_ws/install/turtlebot3_autorace_tracking'
