# ***************************************************************************************
# *
# *    Author: TzeChing Luk
# *    Date: 2023-09-28
# *
# ***************************************************************************************

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from tier4_vehicle_msgs.msg import ActuationCommandStamped
import struct
import math

class DummyPublisher(Node):

    def __init__(self):
        super().__init__('dummy_publisher')
        self.tire_angle_deg = 0.0
        self.steer_ratio = 16.48571

        self.publisher_ = self.create_publisher(UInt8MultiArray, '/serial_write', 10)
        self.actuation_cmd_subscription_ = self.create_subscription(ActuationCommandStamped, '/control/command/actuation_cmd', self.steer_callback, 1)
        self.actuation_cmd_subscription_
        timer_period = 0.1  # [s]
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.speed_control_cmd(3)  # Set the rps of motor, e.g. 3 means 3 revolution per seconds

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

    def steer_callback(self, msg: ActuationCommandStamped):
        self.tire_angle_deg = math.degrees(msg.actuation.steer_cmd)

    def speed_control_cmd(self, rps: float):
        msg_speed = UInt8MultiArray()
        step_pre_seconds = int(rps * 51200)
        speed_cmd_data = struct.pack('!7B', 0x01, 0x10, 0x00, 0x26, 0x00, 0x02, 0x04)
        byte_step_pre_seconds = struct.pack('!I', step_pre_seconds)
        crc = self.calculate_crc16((speed_cmd_data + byte_step_pre_seconds))
        msg_speed.data = speed_cmd_data + byte_step_pre_seconds + crc
        self.publisher_.publish(msg_speed)
        self.get_logger().info(f'Target speed [rps]: {rps}')
        self.get_logger().info(f'Publishing: "{msg_speed.data}"')

    def tire_to_steer_convert(self, tire_angle_deg: float) -> float:
        target_steer = tire_angle_deg * self.steer_ratio
        return target_steer

    def steer_control_cmd(self):
        msg_steer = UInt8MultiArray()
        target_steer = self.tire_to_steer_convert(self.tire_angle_deg)
        angle_to_pulse = int(abs(target_steer)/360 * 51200)  # 51200 pulse for 1 revolution.
        byte_cap_angle_to_pulse = struct.pack('!I', min(angle_to_pulse, 0xFFFFFF))[-3:]  # Max: 0xFFFFFF -> 16777215
        if target_steer >= 0:
            steer_direction_data = 0x00
        else:
            steer_direction_data = 0x80
        steer_cmd_data = struct.pack('!7B', 0x01, 0x10, 0x00, 0x2B, 0x00, 0x02, 0x04)
        steer_angle_data = struct.pack('!B3s', steer_direction_data, byte_cap_angle_to_pulse)
        crc = self.calculate_crc16((steer_cmd_data + steer_angle_data))
        msg_steer.data = steer_cmd_data + steer_angle_data + crc
        self.publisher_.publish(msg_steer)
        self.get_logger().info(f'Target steer: {target_steer}')
        self.get_logger().info(f'Publishing: "{msg_steer.data}"')

    def timer_callback(self):
        self.steer_control_cmd()


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