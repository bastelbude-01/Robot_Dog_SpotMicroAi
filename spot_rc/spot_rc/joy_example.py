#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

class SnowblowerController(Node):
    def __init__(self):
        super().__init__('snowblower_joy')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.case_pub_ = self.create_publisher(Int8, '/salt', 10)
        self.fr_speed_pub_ = self.create_publisher(Int8, '/blower_speed', 10)
        self.fras_speed = 0

    def joy_callback(self, msg):
        # Check if the message has enough buttons
        if len(msg.buttons) >= 4:
            button_A = msg.buttons[0]
            button_B = msg.buttons[1]
            button_X = msg.buttons[2]
            button_Y = msg.buttons[3]

            # Handle button presses and publish corresponding values on /LEDs topic
            if button_A == 1:
                self.blower_case_(1)
            elif button_B == 1:
                self.blower_case_(0)
            elif button_X == 1:
                self.blower_case_(2)
            elif button_Y == 1:
                self.blower_case_(3)

            # Handle servo control based on axes 6 and 7
            axis_6_value = msg.axes[6]
            axis_7_value = msg.axes[7]

            if axis_7_value == 1:
                self.fras_speed = 15
            elif axis_7_value == -1:
                self.fras_speed = 0
            elif axis_6_value == 1:
                self.fras_speed = min(self.fras_speed + 1, 45)
            elif axis_6_value == -1:
                self.fras_speed = max(self.fras_speed - 1, 0)

            # Publish the servo position
            fr_msg = Int8()
            fr_msg.data = self.fras_speed
            self.fr_speed_pub_.publish(fr_msg)

    def blower_case_(self, value):
        case_msg = Int8()
        case_msg.data = value
        self.case_pub_.publish(case_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SnowblowerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
