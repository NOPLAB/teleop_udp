#!/usr/bin/env python3
"""
ROS2 node for receiving controller data via UDP.
Publishes sensor_msgs/Joy messages.
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


class UdpReceiverNode(Node):
    def __init__(self):
        super().__init__('udp_receiver')

        # Declare parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 12345)
        self.declare_parameter('frame_id', 'joy')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publisher
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Setup UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(0.1)  # 100ms timeout for non-blocking
        self.socket.bind((self.host, self.port))

        self.get_logger().info(f'Listening on {self.host}:{self.port}')

        # Timer for receiving data
        self.timer = self.create_timer(0.01, self.receive_data)  # 100Hz

    def receive_data(self):
        try:
            # Receive controller packet (8 bytes)
            # uint32_t buttons + uint8_t lx, ly, rx, ry
            data, addr = self.socket.recvfrom(8)
            if len(data) < 8:
                return

            # Unpack: little-endian uint32 + 4 uint8
            buttons, lx, ly, rx, ry = struct.unpack('<IBBBB', data)

            # Create Joy message
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.header.frame_id = self.frame_id

            # Convert analog sticks to -1.0 to 1.0 range
            # Center is 128, range is 0-255
            # Layout matches standard joy_node (PS4/Xbox style):
            # axes[0]: Left stick X
            # axes[1]: Left stick Y
            # axes[2]: L2 trigger (not available via UDP, default 0)
            # axes[3]: Right stick X
            # axes[4]: Right stick Y
            # axes[5]: R2 trigger (not available via UDP, default 0)
            joy_msg.axes = [
                (lx - 128) / 128.0,   # Left stick X
                -(ly - 128) / 128.0,  # Left stick Y (inverted)
                0.0,                  # L2 trigger (placeholder)
                (rx - 128) / 128.0,   # Right stick X
                -(ry - 128) / 128.0,  # Right stick Y (inverted)
                0.0,                  # R2 trigger (placeholder)
            ]

            # Convert buttons to list of 0/1
            joy_msg.buttons = [
                1 if buttons & flag else 0
                for flag in BUTTON_FLAGS
            ]

            self.joy_pub.publish(joy_msg)

        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().warn(f'Error receiving data: {e}')

    def destroy_node(self):
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpReceiverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
