# MPC Controller Documentation

## Overview
This document explains the implementation of a Model Predictive Control (MPC) system for a ROS 2 node using Python. The controller minimizes errors in angle and speed by solving an optimization problem with constraints using `scipy.optimize.minimize`.

---

## Key Components

### 1. Error Calculation
- **Angle Error** (`angle_error`): Represents the difference between the desired and actual angles. It is generated as a random value between `-π` and `π` radians for this example.
- **Speed Error** (`speed_error`): Represents the difference between the desired and actual speeds. It is generated as a random value between `-1.5 m/s` and `1.5 m/s` for this example.

### 2. Control Limits
- **Speed Limits**: `-1.2 m/s` to `1.5 m/s`.
- **Angle Limits**: `-π` to `π` radians.

### 3. Cost Function
The cost function is used to minimize the errors while penalizing excessive control effort. The mathematical representation is:

\[
J(u) = q_{\text{angle}} (\text{angle\_error} + u_1)^2 + q_{\text{speed}} (\text{speed\_error} + u_2)^2 + r_{\text{control}} (u_1^2 + u_2^2)
\]

Where:
- \( u = [u_1, u_2] \): Control variables, \( u_1 \) is the angle correction, and \( u_2 \) is the speed correction.
- \( q_{\text{angle}}, q_{\text{speed}} \): Weights for the angle and speed errors, respectively.
- \( r_{\text{control}} \): Weight for penalizing the control effort.

The optimization minimizes \( J(u) \).

### 4. Optimization Constraints
The optimization problem is solved with bounds:
- \( -\pi \leq u_1 \leq \pi \)
- \( -1.2 \leq u_2 \leq 1.5 \)

The `scipy.optimize.minimize` function is used with the Sequential Least Squares Programming (SLSQP) method to solve this problem.

---

## Implementation Details

### ROS 2 Node
The node runs at a frequency of 10 Hz. In each update cycle:
1. **Errors** are retrieved using `get_angle_error()` and `get_speed_error()`.
2. **Optimization** is performed to compute control corrections using the cost function and constraints.
3. **Corrections** are applied to minimize the errors and logged for analysis.

### Code Snippet for Optimization
```python
# Define cost function
    def cost_function(u):
        angle_correction, speed_correction = u
        cost = (
            q_angle * (angle_error + angle_correction)**2 +
            q_speed * (speed_error + speed_correction)**2 +
            r_control * (angle_correction**2 + speed_correction**2)
        )
        return cost

# Solve optimization problem
result = minimize(
    cost_function,
    x0=[0.0, 0.0],  # Initial control guess
    bounds=bounds,
    method='SLSQP'
)
```

### Handling Optimization Results
- If the optimization is successful, the control corrections are applied.
- If the optimization fails, the system logs a warning and uses zero corrections as fallback values.

---

## Logging and Output
The corrections for angle and speed are logged at each cycle:
```python
self.get_logger().info(f'Angle Correction: {angle_correction}, Speed Correction: {speed_correction}')
```

---

## Mathematical Summary
- The controller minimizes the cost function \( J(u) \) with constraints.
- Weights \( q_{\text{angle}}, q_{\text{speed}}, r_{\text{control}} \) are used to balance error correction and control effort.
- Constraints ensure that the control actions remain within physical limits.

---

## Running the Node
1. Save the script in your ROS 2 package.
2. Run the node using:
   ```bash
   ros2 run <package_name> mpc_controller.py
   ```

Replace `<package_name>` with the appropriate package name.

---

## Conclusion
This implementation demonstrates the use of MPC in a ROS 2 framework with optimization techniques. It can be extended further by integrating real-world sensor data and actuators for practical applications.
