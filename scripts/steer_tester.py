import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SteerTester(Node):
    def __init__(self):
        super().__init__("steer_tester")
        self.publisher_ = self.create_publisher(Float32, "/tester/steer", 1)

    def run(self):
        while rclpy.ok():
            value = float(input(f"target steer: "))
            msg = Float32(data=value)
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    steer_tester = SteerTester()
    steer_tester.run()

    steer_tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
