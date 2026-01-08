#!/usr/bin/env python3
"""
ROS2 node for sending Joy messages via UDP.
Subscribes to sensor_msgs/Joy and sends as UDP packets.
"""

import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


# Button bit flags (same as SCE_CTRL_* values)
BUTTON_SELECT   = 0x00000001
BUTTON_L3       = 0x00000002
BUTTON_R3       = 0x00000004
BUTTON_START    = 0x00000008
BUTTON_UP       = 0x00000010
BUTTON_RIGHT    = 0x00000020
BUTTON_DOWN     = 0x00000040
BUTTON_LEFT     = 0x00000080
BUTTON_L        = 0x00000100
BUTTON_R        = 0x00000200
BUTTON_L2       = 0x00000400
BUTTON_R2       = 0x00000800
BUTTON_TRIANGLE = 0x00001000
BUTTON_CIRCLE   = 0x00002000
BUTTON_CROSS    = 0x00004000
BUTTON_SQUARE   = 0x00008000

# Button order for Joy message (index mapping)
BUTTON_FLAGS = [
    BUTTON_CROSS,     # 0: A / Cross
    BUTTON_CIRCLE,    # 1: B / Circle
    BUTTON_SQUARE,    # 2: X / Square
    BUTTON_TRIANGLE,  # 3: Y / Triangle
    BUTTON_L,         # 4: LB / L
    BUTTON_R,         # 5: RB / R
    BUTTON_L2,        # 6: LT / L2
    BUTTON_R2,        # 7: RT / R2
    BUTTON_SELECT,    # 8: Back / Select
    BUTTON_START,     # 9: Start
    BUTTON_L3,        # 10: L3
    BUTTON_R3,        # 11: R3
    BUTTON_UP,        # 12: D-Pad Up
    BUTTON_DOWN,      # 13: D-Pad Down
    BUTTON_LEFT,      # 14: D-Pad Left
    BUTTON_RIGHT,     # 15: D-Pad Right
]


class UdpSenderNode(Node):
    def __init__(self):
        super().__init__('udp_sender')

        # Declare parameters
        self.declare_parameter('target_host', '127.0.0.1')
        self.declare_parameter('target_port', 12345)

        self.target_host = self.get_parameter('target_host').get_parameter_value().string_value
        self.target_port = self.get_parameter('target_port').get_parameter_value().integer_value

        # Setup UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info(f'Sending to {self.target_host}:{self.target_port}')

    def joy_callback(self, msg: Joy):
        try:
            # Convert buttons from list to bitmask
            buttons = 0
            for i, flag in enumerate(BUTTON_FLAGS):
                if i < len(msg.buttons) and msg.buttons[i]:
                    buttons |= flag

            # Convert axes from -1.0 to 1.0 range to 0-255 (center 128)
            # axes[0]: Left stick X
            # axes[1]: Left stick Y (inverted)
            # axes[2]: Right stick X
            # axes[3]: Right stick Y (inverted)
            lx = int(msg.axes[0] * 128 + 128) if len(msg.axes) > 0 else 128
            ly = int(-msg.axes[1] * 128 + 128) if len(msg.axes) > 1 else 128
            rx = int(msg.axes[2] * 128 + 128) if len(msg.axes) > 2 else 128
            ry = int(-msg.axes[3] * 128 + 128) if len(msg.axes) > 3 else 128

            # Clamp values to 0-255
            lx = max(0, min(255, lx))
            ly = max(0, min(255, ly))
            rx = max(0, min(255, rx))
            ry = max(0, min(255, ry))

            # Pack: little-endian uint32 + 4 uint8
            data = struct.pack('<IBBBB', buttons, lx, ly, rx, ry)
            self.socket.sendto(data, (self.target_host, self.target_port))

        except Exception as e:
            self.get_logger().warn(f'Error sending data: {e}')

    def destroy_node(self):
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
