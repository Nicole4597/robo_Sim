import json

import rclpy
from my_interfaces.msg import ActorPosition
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(String, '/my_actor_info', 10)    # publishes my ros1 json
        
        self.subscription_1 = self.create_subscription(ActorPosition, '/actor_position', self.position_callback, 10) # listens for actor pos (Lorenzo)
        self.subscription_2 = self.create_subscription(String, '/scenic_action', self.action_callback, 10)       # listens for scenic action (Lorenzo)

        self.info = {}
        self.update_pos = False
        self.update_act = False
        
        self.timer = self.create_timer(0.25, self.publish)   # check new data every 0.25 seconds
        self.get_logger().info("started, waiting for messages...")


    def position_callback(self, msg):
        self.get_logger().info(f"I received {msg}")
        self.info["actorPosX"] = msg.actor_x
        self.info["actorPosY"] = msg.actor_y
        self.update_pos = True


    def action_callback(self, msg):
        self.get_logger().info(f"I received {msg}")
        self.info["scenicAction"] = msg.data
        self.update_act = True


    def publish(self):
        if not (self.update_pos and self.update_act):
            return
        msg = String()
        msg.data = json.dumps(self.info)
        self.publisher.publish(msg)
        self.update_pos = False
        self.update_act = False
        self.get_logger().info(f"I published {msg}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
