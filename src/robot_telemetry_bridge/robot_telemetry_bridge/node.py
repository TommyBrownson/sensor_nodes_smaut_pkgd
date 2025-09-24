"""
Telemetry Data Bridge - Parses zmq messages and publishes data as ROS2 messages
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_telemetry_common.telemetry_io import TelemetrySubscriber

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        self.declare_parameter('endpoint', 'tcp://192.168.232.50:35000')
        endpoint = self.get_parameter('endpoint').get_parameter_value().string_value

        self.subscriber = TelemetrySubscriber(endpoint=endpoint)
        self.publisher = self.create_publisher(String, '/robot/telemetry_raw', 10)


        self.timer = self.create_timer(0.01, self.timer_callback)  #

    def timer_callback(self):
        data = self.subscriber.receive_zmq()

        if data is None:
            return

        """Parse robot model data from ZMQ message"""
        # Fix typo in power_management field key if present
        pm_raw = data.get("power_management", {})
        # rename 'batery_voltage' to 'battery_voltage'
        if 'batery_voltage' in pm_raw:
            pm_raw['battery_voltage'] = pm_raw.pop('batery_voltage')

        msg = String()
        msg.data = json.dumps(data)  # simple, human-readable
        self.publisher.publish(msg) 

    def destroy(self):
        try:
            self.subscriber.close()
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    telemetry_bridge = TelemetryBridge()
    
    try:
        rclpy.spin(telemetry_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        telemetry_bridge.destroy()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
