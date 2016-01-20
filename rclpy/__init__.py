import rclpy._rclpy__rmw_opensplice_cpp as _rclpy

from rclpy.node import Node

def init(args):
    return _rclpy.rclpy_init(args)

def create_node(node_name):
    node_handle = _rclpy.rclpy_create_node(node_name)
    return Node(node_handle)

def spin(node):
    wait_set = _rclpy.rclpy_get_zero_initialized_wait_set()

    _rclpy.rclpy_wait_set_init(wait_set, len(node.subscriptions), 0, 0)

    while True:
        _rclpy.rclpy_wait_set_clear_subscriptions(wait_set)
        for subscription in node.subscriptions:
            _rclpy.rclpy_wait_set_add_subscription(wait_set, subscription.subscription_handle)
            _rclpy.rclpy_wait(wait_set)

            msg = _rclpy.rclpy_take(subscription.subscription_handle, subscription.msg_type)

            if msg:
                subscription.callback(msg)
