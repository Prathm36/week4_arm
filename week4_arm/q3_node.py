#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math

class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')
        self.subscription = self.create_subscription(
            Point,
            '/end_effector_position',
            self.listener_callback,
            10)
        self.subscription
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/joint_angles_goal',
            10
        )

    def listener_callback(self, msg):
        current_x = msg.x
        current_y = msg.y
        self.get_logger().info(f"Cuurent position: {current_x}, {current_y}")
        '''Rest of function is executed only after input is recieved.'''
        input_data = self.waitForInput()
        target = [current_x, current_y]
        if input_data[0] == 'x':
            target[0] += input_data[1]
        else:
            target[1] += input_data[1]
        dist = math.sqrt(target[0]**2 + target[1]**2)
        max_reach = 3.5
        min_reach = 0.5
        if dist < min_reach:
            self.get_logger().info("Target too close.")
            return
        elif dist > max_reach:
            self.get_logger().info("Target too far.")
            return
        
        '''Calculation of the angles required for the End-effector at the target point.'''
        l1 = 2
        l2 = 1.5
        target_x = target[0]
        target_y = target[1]
        theta2 = math.acos((target_x**2 + target_y**2 - 4 - 2.25)/(2*l1*l2))
        theta1 = math.atan2(target_y, target_x) - math.atan((l2*math.sin(theta2))/(l1+l2*math.cos(theta2)))
        angles = Float64MultiArray()
        angles.data = [theta1, theta2]
        self.publisher_.publish(angles)
        self.get_logger().info(f"Publishing: {angles.data[0]}, {angles.data[1]}")
    
    '''Function for the input and so that the program pauses until the input from user is recieved.'''
    def waitForInput(self):
        while True:
            direction = input("Enter direction x or y: ")
            distance = float(input("Enter distance in that direction: "))
            if direction == 'x' or direction == 'y' or abs(distance) <= 0.5:
                break
            else:
                print("Invalid input")
        return([direction, distance])

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MoveArm()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()