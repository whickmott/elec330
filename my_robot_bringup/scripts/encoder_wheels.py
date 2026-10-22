#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from smbus import SMBus
from std_msgs.msg import Float64MultiArray


class EncoderWheels(Node):
    def __init__(self):
        super().__init__('encoder_wheels')
        self.declare_parameter('from_joint_states', False)
        self.declare_parameter('radians_per_count', 0.1)
        self.bus = None
        self.angles = None
        self.joints = ['base_left_wheel_joint', 'base_right_wheel_joint']
        self.publisher = self.create_publisher(
            Float64MultiArray, '/sim/wheel_position_controller/commands', 10)

        if self.get_parameter('from_joint_states').value:
            self.subscription = self.create_subscription(
                JointState,
                '/joint_state_broadcaster/joint_states',
                self.receive,
                qos_profile_sensor_data)
        else:
            self.scale = self.get_parameter('radians_per_count').value
            self.bus = SMBus(1)
            self.addresses = [0x20, 0x24]
            for address in self.addresses:
                self.bus.write_i2c_block_data(
                    address, 7,
                    [127, 255, 255, 255, 128, 0, 0, 0, 0, 0, 0, 1])
            self.previous = [self.read_count(a) for a in self.addresses]
            self.angles = [0.0, 0.0]

        self.create_timer(0.05, self.update)

    def receive(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in self.joints):
            angles = [positions[name] for name in self.joints]
            if all(math.isfinite(angle) for angle in angles):
                self.angles = angles

    def read_count(self, address):
        data = self.bus.read_i2c_block_data(address, 3, 4)
        return int.from_bytes(bytes(data), 'big', signed=True)

    def update(self):
        if self.bus is not None:
            try:
                counts = [self.read_count(a) for a in self.addresses]
            except OSError as error:
                self.get_logger().error(str(error), throttle_duration_sec=5.0)
                return
            for i, count in enumerate(counts):
                difference = (count - self.previous[i] + 2**31) % 2**32 - 2**31
                self.angles[i] += difference * self.scale
                self.previous[i] = count

        if self.angles is not None:
            self.publisher.publish(Float64MultiArray(data=list(self.angles)))


def main():
    rclpy.init()
    node = None
    try:
        node = EncoderWheels()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.bus is not None:
                node.bus.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()