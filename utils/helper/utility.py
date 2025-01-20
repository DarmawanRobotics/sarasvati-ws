from rclpy.node import Node

def clamp(value: float, bound: float) -> float:
    """Clamp the value within the specified bound."""
    return max(min(value, bound), -bound)

def print_parameters(node: Node, namespace: str):
    """Utility function to print all parameters of a node."""
    parameters = node.list_parameters([], depth=1).names
    param_values = {param: node.get_parameter(param).value for param in parameters if param.startswith(namespace)}

    print(f"Parameters in namespace '{namespace}':")
    for param, value in param_values.items():
        print(f"  {param}: {value}")
    

