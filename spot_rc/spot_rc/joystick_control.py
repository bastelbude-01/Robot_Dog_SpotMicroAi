import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from Control import *



class DogCommands(Node):
    def __init__(self):
        super().__init__('dog_commands')
        self.subscriptions = self.create_subscription(Twist, '/cmd_vel', self.dog_callback, 10)

    def dog_callback(self, msg):
        x = msg.linar.x
        y = msg.angular.z
        
        if x >= 0.3:
            control.forward()


def main(args=Node):
    rclpy.init(args=args)

    node_ = DogCommands()

    rclpy.spin(node_)
    DogCommands.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
