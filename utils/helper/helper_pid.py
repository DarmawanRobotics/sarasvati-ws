import numpy as np
from utility import clamp, print_parameters
from rclpy.node import Node

class HelperPID
    def __init__(self, node : Node,  name: str = None):
        self._name = name
        self._error_before = 0.0
        self._error_accumulation = 0.0
        self._kP = []
        self._kI = []
        self._kD = []

        self.node = node
        self.init_parameters()

    def get_membership_idx(self, error: float) -> int:
        """Determine the membership index based on error magnitude."""
        step = self.output_bound / self.membership_amount
        return min(self.membership_amount - 1, int(abs(error) / step))
  

    def set_pid(self, max_kP: float, max_kI: float, max_kD: float):
        """Set PID constants with calculated membership levels based on maximum values."""
        self._kP = np.linspace(0, max_kP, self.membership_amount).tolist()
        self._kI = np.linspace(0, max_kI, self.membership_amount).tolist()
        self._kD = np.linspace(0, max_kD, self.membership_amount).tolist()

    def reset(self):
        """Reset error states."""
        self._error_before = 0.0
        self._error_accumulation = 0.0

    def update(self, error: float) -> float:
        """Calculate and return the PID output based on the error and time step."""
        if not self._name or not (self._kP and self._kI and self._kD):
            self.reset()
            return 0.0

        idx = self.get_membership_idx(error)
        proportional = self._kP[idx] * error
        derivative = self._kD[idx] * (error - self._error_before)
        self._error_accumulation = clamp(self._error_accumulation + error,self.accumulation_bound)
        integral = self._kI[idx] * self._error_accumulation

        output = clamp(proportional + integral + derivative, self.output_bound)
        self._error_before = error
        return output
    
    def init_parameters(self):
        self.node.declare_parameters(
            namespace=self._name,
            parameters=[
                ('membership_amount', 2),
                ('output_bound', 1.0),
                ('accumulation_bound', 1.0),
                ('max_kP', 1.0),
                ('max_kI', 0.3),
                ('max_kD', 0.02),
            ]
        )

        self.membership_amount = self.node.get_parameter(f'{self._name}.membership_amount').value
        self.output_bound = self.node.get_parameter(f'{self._name}.output_bound').value
        self.accumulation_bound = self.node.get_parameter(f'{self._name}.accumulation_bound').value
        
        max_kP = self.node.get_parameter(f'{self._name}.max_kP').value
        max_kI = self.node.get_parameter(f'{self._name}.max_kI').value
        max_kD = self.node.get_parameter(f'{self._name}.max_kD').value
        
        self.set_pid(max_kP, max_kI, max_kD)
        self.reset()
        print_parameters(self.node, self.name)