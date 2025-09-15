"""
Helper for intaking ZMQ for different sensors
"""

import time
import zmq
import msgpack

class TelemetrySubscriber:
    """
    Class to handle ZMQ subscription and message parsing
    """

    def __init__(self, endpoint: str = "tcp://192.168.232.50:35000", topic: str = ""):
        """
        Initialize the TelemetrySubscriber with a ZMQ SUB socket.
        """
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.SUB)
        self.endpoint = endpoint
        self.socket.connect(self.endpoint)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)#"")
        self.socket.setsockopt(zmq.RCVTIMEO, 50)  # 50ms timeout

    def receive_zmq(self):
        """
        Receive a message from the ZMQ socket.
        """
        try:
            msg = self.socket.recv(zmq.NOBLOCK)
        except zmq.Again:
            # No message available, continue
            return None
        except Exception:
            #self.get_logger().error(f"Failed to recieve message: {e}")
            return None

        try: 
            unpacked = msgpack.unpackb(msg, raw=False)
            return unpacked
        except Exception:
            #self.get_logger().error(f"Failed to unpack message: {e}")
            return None


    def close(self):
        """Clean up ZMQ resources"""
        if hasattr(self, 'socket'):
            self.socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()
