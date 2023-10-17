# ***************************************************************************************
# *
# *    Author: TzeChing Luk
# *    Date: 2023-10-17
# *
# ***************************************************************************************

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32

import struct

class DummyPublisher(Node):

    def __init__(self):
        super().__init__('dummy_publisher')
        self.target_steer = 0.0

        self.publisher_ = self.create_publisher(UInt8MultiArray, '/serial_write', 10)
        self.subscription_ = self.create_subscription(Float32, '/tester/steer', self.steer_callback, 1)
        self.subscription_
        timer_period = 0.1  # [s]
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def calculate_crc16(self, data: list[int]):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001  # 0xA001 is the bit-reversed form of the polynomial 0x8005
                else:
                    crc >>= 1
        crc_bytes = bytes([(crc & 0xFF), ((crc >> 8) & 0xFF)])
        return crc_bytes

    def steer_callback(self, msg: Float32):
        self.target_steer = msg.data
        print(f"Set target steer: {self.target_steer}")

    def timer_callback(self):
        msg_steer = UInt8MultiArray()
        angle_to_pulse = int(abs(self.target_steer)/360 * 51200)  # 51200 pulse for 1 revolution.
        byte_cap_angle_to_pulse = struct.pack('!I', min(angle_to_pulse, 0xFFFFFF))[-3:]  # Max: 0xFFFFFF -> 16777215
        if self.target_steer >= 0:
            steer_direction_data = 0x00
        else:
            steer_direction_data = 0x80
        steer_cmd_data = struct.pack('!7B', 0x01, 0x10, 0x00, 0x2B, 0x00, 0x02, 0x04)
        steer_angle_data = struct.pack('!B3s', steer_direction_data, byte_cap_angle_to_pulse)
        crc = self.calculate_crc16((steer_cmd_data + steer_angle_data))
        msg_steer.data = steer_cmd_data + steer_angle_data + crc
        # msg_steer.data = [0x01, 0x10, 0x00, 0x2B, 0x00, 0x02, 0x04, 0x00, 0x00, 0xC8, 0x00, 0xE7, 0xC4]  # Test command (1 complete revolution)
        self.publisher_.publish(msg_steer)
        self.get_logger().info(f'Publishing: "{msg_steer.data}"')


def main(args=None):
    rclpy.init(args=args)

    dummy_publisher = DummyPublisher()

    rclpy.spin(dummy_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dummy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()