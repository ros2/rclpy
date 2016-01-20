class Subscription:

    def __init__(self, subscription_handle, msg_type, topic, callback, qos_profile):
        self.subscription_handle = subscription_handle
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos_profile = qos_profile
