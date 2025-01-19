#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import ActuatorMotors, OffboardControlMode, VehicleControlMode
from rclpy.qos import QoSProfile


class PX4IONode(Node):
    def __init__(self):
        """Initialize the PX4 IO node with subscriptions, publishers, and timers."""
        super().__init__('px4_io_node')
        self.is_armed = True
        self.is_offboard = False
        self.actuator_msg = ActuatorMotors()

        qos_profile = QoSProfile(depth=10)

        # ========== SUBSCRIBERS ================
        self.create_subscription(Float32MultiArray, 'sasv/actuator/thruster', self.thruster_callback, qos_profile)
        self.create_subscription(Float32MultiArray, 'sasv/actuator/servo', self.servo_callback, qos_profile)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_callback, qos_profile)

        # ========== PUBLISHERS ================
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.actuator_pub = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)

        # ========== TIMERS ================
        self.create_timer(0.002, self.publish_actuator) # 400Hz
        self.create_timer(0.5, self.publish_offboard)   # 2Hz

        self.get_logger().info('PX4 IO node started')

    def thruster_callback(self, msg):
        """Callback function for processing thruster messages."""
        for i in range(min(4, len(msg.data))):
            self.actuator_msg.control[i] = msg.data[i]

    def servo_callback(self, msg):
        """Callback function for processing servo messages."""
        for i in range(min(2, len(msg.data))):
            self.actuator_msg.control[i + 4] = -msg.data[i]

    def control_mode_callback(self, msg):
        """Callback function for processing vehicle control mode messages."""
        self.is_offboard = msg.flag_control_offboard_enabled
        self.is_armed = not msg.flag_armed

    def publish_actuator(self):
        """Publish actuator messages if the system is armed and in offboard mode."""
        if self.is_armed:
            self.get_logger().warn('Not armed')
            return
        if not self.is_offboard:
            self.get_logger().warn('Not in offboard mode')
            return

        self.actuator_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.actuator_msg.timestamp_sample = self.actuator_msg.timestamp
        self.actuator_msg.reversible_flags = 255
        self.actuator_pub.publish(self.actuator_msg)

    def publish_offboard(self):
        """Publish offboard control mode messages."""
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.direct_actuator = True
        self.offboard_pub.publish(msg)


def main(args=None):
    """Main function to initialize and run the PX4 IO node."""
    rclpy.init(args=args)
    node = PX4IONode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
