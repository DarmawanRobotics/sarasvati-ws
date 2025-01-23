import sys
import os.path
from rclpy.node import Node
from geometry_msgs.msg import Twist

sys.path.append(os.path.abspath('../sarasvati/src/xtras/util'))
from utility import clamp  
from helper_mpc import HelperMPC as MPC

class BasicDrive:
    """Controller for driving mechanics using fuzzy PID."""
    
    def __init__(self, node: Node):
        self.node = node
        self.mpc = MPC(self.node, "basic_drive")
        
        self.last_speed_azimuth = 0
        self.last_speed_diff = 0

        self.thruster_base = 0.5      
        
        self.max_speed = 1.0 
        self.min_speed = -1.0 

    def compute_speed_adj(self, current_speed: float, target_speed: float):
        """Calculate speed adjustments based on target."""
        speed_error = target_speed - current_speed
        speed_adjustment = self.mpc.update(speed_error) 
        
        return clamp(speed_adjustment, self.min_speed, self.max_speed)

    def azimuth_drive(self, current_speed: float, current_yaw: float, target: Twist):
        """Calculate adjustments for azimuth drive."""
        speed_adjustment = self.compute_speed_adj(current_speed, target.linear.x)
        self.last_speed_azimuth += speed_adjustment
        
        return self.last_speed_azimuth, target.angular.z

    def differential_drive(self, current_speed: float, current_yaw: float, target: Twist):
        """Calculate left and right wheel speeds for differential drive."""
        speed_adjustment = self.compute_speed_adj(current_speed, target.linear.x)
        self.last_speed_diff += speed_adjustment
        
        left_wheel_speed = self.last_speed_diff - (target.angular.z * self.thruster_base / 2.0)
        right_wheel_speed = self.last_speed_diff + (target.angular.z * self.thruster_base / 2.0)    
        return left_wheel_speed, right_wheel_speed