from . import topic_if


class Robot_State_Subscriber(topic_if.SubscriberIf):
    def __init__(self, topic_name):
        super().__init__(topic_name, self.state_listener_callback)
        self.current_state = None
    def state_listener_callback(self, msg):
        self.current_state = msg.state