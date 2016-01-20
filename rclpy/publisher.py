import rclpy._rclpy__rmw_opensplice_cpp as _rclpy

class Publisher:
    def __init__(self, publisher_handle, msg_type, topic, qos_profile):
        self.publisher_handle = publisher_handle
        self.msg_type = msg_type
        self.topic = topic
        self.qos_profile = qos_profile

    def publish(self, msg):
        _rclpy.rclpy_publish(self.publisher_handle, msg)
