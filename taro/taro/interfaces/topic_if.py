import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SubscriberIf(Node):

    def __init__(self, name, callback_handle: callable):
        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            'topic',
            callback_handle,
            10)
        self.subscription  # prevent unused variable warning