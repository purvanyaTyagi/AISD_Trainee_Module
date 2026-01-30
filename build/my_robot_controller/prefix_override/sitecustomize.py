import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tss8117/Desktop/tirth_ws/AISD_Trainee_Module/install/my_robot_controller'
