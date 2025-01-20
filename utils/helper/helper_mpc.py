import rclpy
from rclpy.node import Node
from math import pi
import numpy as np
from scipy.optimize import minimize
from utility import print_parameters

class HelperMPC():
    def __init__(self, node: Node, name: str = None):
        self.num_variabel = 1
        self.limits = [(-1.0, 1.0)] 
        self.q_weights = [0.9]  
        self.r_control = 0.01  

        self.node = node
        self.name = name if name else "mpc_controller"
        
        self.init_parameters()

    def update(self, errors):
        """Generalized MPC control to handle dynamic input variables."""
        
        if not (len(errors) == self.num_variabel):
            raise ValueError("Length of errors must match num_variabel.")

        def cost_function(u):
            """Cost function to minimize."""
            cost = 0.0
            for i in range(len(errors)):
                cost += self.q_weights[i] * (errors[i] + u[i])**2
            cost += self.r_control * sum(u_i**2 for u_i in u)
            return cost

        x0 = [0.0] * len(errors)  
        
        result = minimize(
            cost_function,
            x0=x0, 
            bounds=self.limits, 
            method='SLSQP'
        )

        if result.success:
            return result.x
        else:
            self.node.get_logger().warn('Optimization failed! Using zero corrections.')
            return [0.0] * len(errors)

    def init_parameters(self):
        """Initialize and declare ROS2 parameters."""
        self.node.declare_parameters(
            namespace=self.name,
            parameters=[
                ("num_variabel", self.num_variabel),
                ("limits", [list(limit) for limit in self.limits]),
                ("q_weights", self.q_weights),
                ("r_control", self.r_control),
            ]
        )
        self.num_variabel = self.node.get_parameter(f"{self.name}.num_variabel").value
        self.limits = [tuple(limit) for limit in self.node.get_parameter(f"{self.name}.limits").value]
        self.q_weights = self.node.get_parameter(f"{self.name}.q_weights").value
        self.r_control = self.node.get_parameter(f"{self.name}.r_control").value
        print_parameters(self.node, self.name)