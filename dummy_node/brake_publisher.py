# ***************************************************************************************
# *
# *    Author: TzeChing Luk
# *    Date: 2023-10-17
# *
# ***************************************************************************************

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from tier4_vehicle_msgs.msg import ActuationCommandStamped
import struct


class BrakePublisher(Node):

    def __init__(self):
        super().__init__('brake_publisher')
        self.linear_travel_per_step = 0.03175  # [mm]
        self.max_brake_travel_distance = 88.9  # [mm]
        self.step_angle = 1.8  # [deg]
        self.max_steps = int(self.max_brake_travel_distance / self.linear_travel_per_step)
        self.brake_cmd = 0.0  # Initial

        self.publisher_ = self.create_publisher(UInt8MultiArray, '/serial_write', 10)
        self.actuation_cmd_subscription_ = self.create_subscription(ActuationCommandStamped, '/control/command/actuation_cmd', self.actuation_cmd_callback, 1)
        self.actuation_cmd_subscription_
        timer_period = 1  # [s]
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.speed_control_cmd(3)  # Set the rps of motor, e.g. 3 means 3 revolution per seconds

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

    def actuation_cmd_callback(self, msg: ActuationCommandStamped):
        self.brake_cmd = msg.actuation.brake_cmd

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

    def brake_control_cmd(self):
        """
        200 steps == 1 revolution == 51200 pulses
        
        Assume the applied brake pressure is linear to the piston linear travel distance.
        """
        msg_brake = UInt8MultiArray()
        required_motor_steps = min(int((self.max_brake_travel_distance * self.brake_cmd) / self.linear_travel_per_step), self.max_steps)
        required_motor_pulses = int(abs(required_motor_steps * self.step_angle) / 360 * 51200)
        byte_cap_angle_to_pulse = struct.pack('!I', min(required_motor_pulses, 0xFFFFFF))[-3:]  # Max: 0xFFFFFF -> 16777215
        motor_cmd_data = struct.pack('!7B', 0x01, 0x10, 0x00, 0x2B, 0x00, 0x02, 0x04)
        motor_angle_data = struct.pack('!B3s', 0x00, byte_cap_angle_to_pulse)
        crc = self.calculate_crc16((motor_cmd_data + motor_angle_data))
        msg_brake.data = motor_cmd_data + motor_angle_data + crc
        self.publisher_.publish(msg_brake)
        self.get_logger().info(f'Required motor steps: {required_motor_steps}, Required motor pulses: {required_motor_pulses}')
        self.get_logger().info(f'Publishing: "{msg_brake.data}"')
        

    def timer_callback(self):
        self.brake_control_cmd()


def main(args=None):
    rclpy.init(args=args)

    brake_publisher = BrakePublisher()

    rclpy.spin(brake_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    brake_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()