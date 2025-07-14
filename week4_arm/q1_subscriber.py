#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription
        self.publisher_ = self.create_publisher(
            Point,
            '/end_effector_position',
            10
        )

    def listener_callback(self, msg):
        theta1 = msg.position[1] + 3.14/2
        theta2 = msg.position[2]
        L1 = 2
        L2 = 1.5
        end_effector = Point()
        end_effector.x = L2*math.cos(theta1+theta2) + L1*math.cos(theta1)
        end_effector.y = L2*math.sin(theta1+theta2) + L1*math.sin(theta1)
        end_effector.z = 0.0
        self.publisher_.publish(end_effector)
        self.get_logger().info(f"Publishing: {end_effector.x}, {end_effector.y}")

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()