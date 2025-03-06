import rclpy
import time
from random import random, randrange
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from my_interfaces.msg import TurtleCreation


class SpawnNodeTurtle(Node):
    def __init__(self, node_name: str, speed_to_produce_turtle: float, topic_name: str):
        """_summary_

        Args:
            node_name (str): _description_
            speed_to_produce_turtle (float): a HZ
        """
        super().__init__(node_name)
        self.client_ = self.create_client(Spawn, topic_name)
        # Turtle creation are predefined.
        self.publisher_ = self.create_publisher(TurtleCreation, "turtle_creation", qos_profile=1)
        while not self.client_.wait_for_service(timeout_sec=1):
            self.get_logger().warn(
                f"service {topic_name} not available, waiting again ..."
            )
        self.counter_: int = 0
        self.req = Spawn.Request()
        self.req_about_spawn = TurtleCreation()

    def produce_new_turtle_callback(self):
        self.counter_ += 1
        self.req._x = float(randrange(0, 10))
        self.req._y = float(randrange(0, 10))
        self.req._theta = float(randrange(0, 10))
        self.get_logger().info(
            f"We are creating another one turtle: x, y {self.req._x, self.req._y}, theta {self.req._theta}"
        )
        return self.client_.call_async(self.req)

    def notify_about_creation(self):
        self.req_about_spawn.x = self.req._x
        self.req_about_spawn.y = self.req._y
        self.req_about_spawn.theta = self.req._theta
        self.get_logger().info(
            f"We pushed a message about turtle creation: x, y {self.req._x, self.req._y}, theta {self.req._theta}"
        )
        self.publisher_.publish(self.req_about_spawn)


def main():
    rclpy.init()
    main_turtle = SpawnNodeTurtle("spawn_turtle_node", 1, "spawn")
    while True:
        future = main_turtle.produce_new_turtle_callback()
        rclpy.spin_until_future_complete(main_turtle, future)
        response = future.result()
        main_turtle.notify_about_creation()
        main_turtle.get_logger().info(f"turtle with name: {response.name}")
        time.sleep(5)

    rclpy.spin(main_turtle)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# self.client_ = self.create_client(AddTwoInts, srv_name=server_name)
