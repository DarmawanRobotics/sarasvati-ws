#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk_custom_interfaces.msg import SetPosition

class DynamixelIONode(Node):
    def __init__(self):
        """Initialize the node, subscriptions, publishers, and timer."""
        super().__init__("camera_servo_node")
        self.servo_pitch = 0
        self.servo = SetPosition()
        
        # ========== ROS UTILS ================
        self.create_subscription(Twist, '/sasv/autonomy/camera/servo', self.camera_servo_callback, 10)
        self.dynamixel_pub = self.create_publisher(SetPosition, 'set_position', 10)
        self.create_timer(0.01, self.update) #100Hz

    def camera_servo_callback(self, msg):
        """Convert received angular velocity to servo position."""
        self.servo_pitch = int(np.rad2deg(msg.angular.x) * 4095 / 360)

    def update(self):
        """Update and publish the servo position commands."""
        self.servo.id = 1
        self.servo.position = self.servo_pitch
        self.dynamixel_pub.publish(self.servo)

        self.get_logger().info(f"SERVO PITCH: {self.servo_pitch}")

def main(args=None):
    """Initialize and run the node."""
    rclpy.init(args=args)
    camera_servo_node = DynamixelIONode()
    rclpy.spin(camera_servo_node)
    camera_servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
