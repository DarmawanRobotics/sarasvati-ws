import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist

import sys
import os.path

sys.path.append(os.path.abspath('../ares/src/xtras/util'))
from sfuzzy_pid import SFuzzyPID

class XDrive:
    def __init__(self,node : Node):
        self.node = node
        self.orientation_pid = SFuzzyPID(self.node,"orientation_pid")
        self.omege_pid = SFuzzyPID(self.node,"omega_pid")
        self.surge_pid = SFuzzyPID(self.node,"surge_pid")
        self.sway_pid = SFuzzyPID(self.node,"sway_pid")

        self.init_parameters()

    def update(self,target : Twist,current_speed : Twist,current_yaw : float):
        """Calculate adjustments for azimuth drive."""
        first_order_vector = np.array([current_speed.linear.x,current_speed.linear.y])
        translation_vector = np.array([target.linear.x,target.linear.y])

        if np.linalg.norm(translation_vector) < 0.001:
            self.node.get_logger().info("Target is too close")
            return 0,0,0,0
        
        yaw_error = current_yaw - target.angular.z
        if yaw_error < -np.pi:
            yaw_error += 2*np.pi
        if yaw_error > np.pi:
            yaw_error -= 2*np.pi
        
        orientation_error = self.orientation_pid.update(yaw_error)
        omega_error = current_speed.angular.z - orientation_error
        omega_out = self.omega_pid.update(omega_error)


        translation_vector_hat = translation_vector / np.linalg.norm(translation_vector)
        translation_error_vector = first_order_vector - translation_vector_hat * np.linalg.norm(first_order_vector)

        target.linear.x -= self.pid_surge.update(translation_error_vector[0])
        target.linear.y -= self.pid_sway.update(translation_error_vector[1])

        thrust1 = self.speed * (target.linear.x * np.cos(np.pi/4) + target.linear.y * np.sin(np.pi/4)) + omega_out
        thrust2 = self.speed * (target.linear.x  * np.cos(np.pi/4) + target.linear.y* np.sin(np.pi/4)) + omega_out
        thrust3 = self.speed * (target.linear.x  * np.cos(np.pi/4) - target.linear.y* np.sin(np.pi/4)) + omega_out
        thrust4 = self.speed * (target.linear.x  * np.cos(np.pi/4) - target.linear.y * np.sin(np.pi/4)) + omega_out

        return thrust1,thrust2,thrust3,thrust4

    def init_parameters(self):
        self.node.declare_parameters(
            namespace='x_drive',
            parameters=[
                ('speed', 1.0),
                ('alpha', 0.3),
            ]
        )
       
        self.speed = self.node.get_parameter('x_drive.speed').value
        self.alpha = self.node.get_parameter('x_drive.alpha').value
        param_info = (
            "X Drive Parameters:\n"
            f"  - Speed: {self.speed}\n"
            f"  - Alpha: {self.alpha}"
        )
        self.node.get_logger().info(param_info)