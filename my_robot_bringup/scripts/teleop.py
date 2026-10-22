#!/usr/bin/env python3
import os
import select
import termios
import time
import tty

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('motor_teleop')
    publisher = node.create_publisher(
        Float64MultiArray, '/motor_controller/commands', 10)

    keyboard = os.open('/dev/tty', os.O_RDWR)
    settings = termios.tcgetattr(keyboard)

    commands = {
        'w': [-0.5, -0.5],
        's': [0.5, 0.5],
        'a': [0.5, -0.5],
        'd': [-0.5, 0.5],
        ' ': [0.0, 0.0],
    }
    command = [0.0, 0.0]
    last_key = 0.0

    try:
        tty.setcbreak(keyboard)
        print('W/S drive | A/D turn | Space stop | Q quit', flush=True)

        while rclpy.ok():
            if select.select([keyboard], [], [], 0.05)[0]:
                key = os.read(keyboard, 1).decode().lower()
                if key == 'q':
                    break
                command = commands.get(key, [0.0, 0.0])
                last_key = time.monotonic()

            if time.monotonic() - last_key > 0.6:
                command = [0.0, 0.0]

            publisher.publish(Float64MultiArray(data=command))
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                for _ in range(5):
                    publisher.publish(Float64MultiArray(data=[0.0, 0.0]))
                    time.sleep(0.05)
        finally:
            termios.tcsetattr(keyboard, termios.TCSADRAIN, settings)
            os.close(keyboard)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()