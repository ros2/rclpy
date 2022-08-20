from rclpy.impl.implementation_singleton.node import Node
from typing import Any

class Publisher:

    def __init__(
        self,
        node: Node,
        msg_type: Any,
        topic: str,
        qos_profile: Any,
        qos_overriding_options: Any = ...,
        /
    ) -> None: ...

    def __enter__(self) -> None: ...

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        pass

    def publish(self, msg: Any) -> None: ...

    def publish_raw(self, msg: bytes) -> None: ...

    def get_subscription_count(self) -> int: ...

    def get_topic_name(self) -> str: ...

    def destroy_when_not_in_use(self) -> None: ...

    def get_logger_name(self) -> str: ...
