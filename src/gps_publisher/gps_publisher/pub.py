"""
GPS Data Publisher - Publishes robot GPS data as ROS2 messages
"""

from dataclasses import dataclass
import msgpack
import time
import rclpy
import json
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, Vector3Stamped
from std_msgs.msg import Header, Float32, Bool, Int32, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import math


@dataclass
class NavigationGNSS:
    lon: float
    lat: float
    alt: float
    accuracy: float
    num_sats: int
    pdop: float
    hdop: float
    vdop: float
    geoid_height: float
    x_local: float
    y_local: float
    heading: float
    rel_pos_n: float
    rel_pos_e: float
    rel_pos_d: float
    acc_n: float
    acc_e: float
    acc_d: float

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.data_sub = self.create_subscription(String, '/robot/telemetry_raw', self.listener_callback, 10)
        self.gps_publisher = self.create_publisher(NavSatFix, '/robot/gps/fix', 10)

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        
        if 'navigation_gnss' not in data:
            return

        nav_data = NavigationGNSS(**data.get("navigation_gnss", {}))
        info_data = data.get('info', {})

        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_frame"

        # GPS coordinates - Convert from scaled integers to decimal degrees
        # Assuming the incoming data is scaled by 10^7 (common GPS format)
        gps_msg.latitude = nav_data.lat   /  10000000.0  # Convert to decimal degrees
        gps_msg.longitude = nav_data.lon  /  10000000.0  # Convert to decimal degrees
        gps_msg.altitude = nav_data.alt

        if 'robot_status' not in data:
            return
        
        # Status - Set based on the RTK fix status
        if data.get('robot_status').get('rtk_fix', False) == 1:  # RTK fix
            gps_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        elif nav_data.num_sats > 3:   # Regular GPS fix
            gps_msg.status.status = NavSatStatus.STATUS_FIX
        else:  # No fix
            gps_msg.status.status = NavSatStatus.STATUS_NO_FIX  # -1

        # Service - Assume both GPS and Galileo (common for modern receivers in Europe)
        gps_msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GALILEO  # 1 + 8 = 9
        
        # Covariance (diagonal elements based on accuracy and DOP values)
        position_covariance = [0.0] * 9
        variance = (nav_data.accuracy ** 2) * (nav_data.hdop ** 2)
        position_covariance[0] = variance  # East variance
        position_covariance[4] = variance  # North variance
        position_covariance[8] = (nav_data.accuracy ** 2) * (nav_data.vdop ** 2)  # Up variance
        gps_msg.position_covariance = position_covariance
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_publisher.publish(gps_msg)        


    def destroy_node(self):
        """Clean up resources"""
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    gps_publisher = GPSPublisher()
    
    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        gps_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
