import rclpy
from rclpy.node import Node

import rclpy
from std_msgs.msg import String

class SubscriberIf(Node):
    def __init__(self, name, topic_name, msg_type,  callback_handle: callable):
        super().__init__(name)
        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            callback_handle,
            10)
        self.subscription  # prevent unused variable warning
