import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tommy/smaut_sensor_nodes_ws/install/radar_data_listener'
