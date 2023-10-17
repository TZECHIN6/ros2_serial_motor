import rclpy
from rclpy.node import Node
from tier4_vehicle_msgs.msg import ActuationCommandStamped


class DummyActuationPublisher(Node):
    def __init__(self):
        super().__init__("steer_tester")
        self.publisher_ = self.create_publisher(ActuationCommandStamped, "/control/command/actuation_cmd", 10)

    def run(self):
        while rclpy.ok():
            value = float(input(f"target steer: "))
            msg = ActuationCommandStamped()
            msg.actuation.steer_cmd = value
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    dummy_actuation_publisher = DummyActuationPublisher()
    dummy_actuation_publisher.run()

    dummy_actuation_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
