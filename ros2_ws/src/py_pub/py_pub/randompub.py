import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from random import Random, randint, random


class RandomPublisher(Node):
    ACTIONS = [
        "attack",
        "scolding",
        "intimidate",
        "grudge",
        "sharing_happiness",
        "happy_person",
        "satisfaction",
        "sharing_fear",
        "running_away",
        "sharing_sadness",
        "disappointment",
        "surprise",
        "disbelief",
        "astonishment",
        "none"
    ]

    POSITION_MAX = 3


    def __init__(self):
        super().__init__('random_publisher')
        self.publisher = self.create_publisher(String, '/my_actor_info', 10)    # publishes my ros1 json
        self.n_pubs = 0
        self.timer = self.create_timer(0.25, self.publish)   # call publish method every 0.25 seconds


    def publish(self):
        # randomly decide to skip publishing
        if random() < 0.3:
            self.get_logger().info(f"I skipped publishing")
            return
        
        random_data = self.get_random_stuff()
        
        msg = String()
        msg.data = json.dumps(random_data)
        self.publisher.publish(msg)
        self.get_logger().info(f"I published {msg}")
        self.n_pubs += 1


    def get_random_stuff(self):
        if self.n_pubs < 100:
            return {
                "actorPosX" : 0,
                "actorPosY" : 5,
                "scenicAction": "none",
            }
        
        return {
            "actorPosX" : 2 * (random() - 0.5) * RandomPublisher.POSITION_MAX,
            "actorPosY" : 2 * (random() - 0.5) * RandomPublisher.POSITION_MAX,
            "scenicAction": RandomPublisher.ACTIONS[ randint(0, len(RandomPublisher.ACTIONS)-1) ],
        }


def main(args=None):
    rclpy.init(args=args)

    random_publisher = RandomPublisher()

    rclpy.spin(random_publisher)

    random_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
