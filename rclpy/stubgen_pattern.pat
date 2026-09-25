# Injects the imports needed by the custom nb::sig() signatures in the C++ bindings,
# which reference rclpy and action_msgs types by their short names.
# The entry below matches the module-level TypeVar ``T`` and re-declares it verbatim.
# The \from escapes add the imports to the top of the generated stub.
_rclpy_nanobind\.T$:
    \from typing import TypeVar
    \from action_msgs.msg import GoalInfo
    \from action_msgs.msg import GoalStatusArray
    \from action_msgs.srv._cancel_goal import CancelGoal_Request
    \from action_msgs.srv._cancel_goal import CancelGoal_Response
    \from rclpy.subscription import MessageInfo
    \from rclpy.subscription_content_filter_options import ContentFilterOptions
    \from rclpy.type_support import Action
    \from rclpy.type_support import FeedbackMessage
    \from rclpy.type_support import FeedbackT
    \from rclpy.type_support import GetResultServiceRequest
    \from rclpy.type_support import GetResultServiceResponse
    \from rclpy.type_support import GoalT
    \from rclpy.type_support import ImplT
    \from rclpy.type_support import MsgT
    \from rclpy.type_support import ResultT
    \from rclpy.type_support import SendGoalServiceRequest
    \from rclpy.type_support import SendGoalServiceResponse
    \from rclpy.type_support import Srv
    \from rclpy.type_support import SrvRequestT
    \from rclpy.type_support import SrvResponseT
    T = TypeVar("T")
