import rclpy
from rclpy.node import Node

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

class PublisherIf(Node):
    def __init__(self, publisher_name, topic_name, msg_type ):
        super().__init__(publisher_name)
        self.publisher = self.create_publisher(msg_type, 'my_topic', 10)
            
    def publish(self, data):
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)