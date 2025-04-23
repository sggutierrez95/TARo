import topic_if


class Robot_State_If(topic_if.SubscriberIf):
    def __init__(self, topic_name):
        super().__init__(topic_name, self.state_listener_callback)
        self.state = None
    def state_listener_callback(self, msg):
        self.state = msg.state