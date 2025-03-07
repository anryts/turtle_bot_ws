import rclpy
from rclpy.node import Node
from my_interfaces.msg import TurtleCreation
from turtlesim.srv import TeleportAbsolute, Kill


class MainTurtleNode(Node):
    def __init__(self, node_name: str, topic_name: str):
        super().__init__(node_name)
        self.subscriber_ = self.create_subscription(
            TurtleCreation, topic_name, self.turtle_creation_listener_callback, 10
        )
        self.client_ = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        self.kill_client_ = self.create_client(Kill, "kill")
        self.get_logger().info(f"Node with name {node_name} was started")
        self.turtle_location = TurtleCreation()
        self.turtle_kill = Kill.Request()

    def turtle_creation_listener_callback(self, msg) -> None:
        self.get_logger().warn(
            f"We receivedd some message for turtle with next coord \nx:{msg.x} \ny:{msg.y} \ntheta:{msg.theta}"
        )
        self.turtle_location: TurtleCreation = msg
        request = TeleportAbsolute.Request()
        request.x = self.turtle_location.x
        request.y = self.turtle_location.y
        request.theta = self.turtle_location.theta
        self.client_.call_async(request)
        #kill 
        self.turtle_kill.name = self.turtle_location.name
        self.kill_client_.call_async(self.turtle_kill)
        #rclpy.spin_until_future_complete(self, future)


def main():
    rclpy.init()
    node = MainTurtleNode("MainTurtle", "turtle_creation")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
