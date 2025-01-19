#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool

class STMCommunicationNode(Node):
    def __init__(self):
        super().__init__('stm_io_node')

        self.init_parameters()
        self.initialize_serial_connection()

        self.pump_state = False
        self.launcher_state = False

        # ========== SUBSCRIBERS ================
        self.create_subscription(Bool, 'cognition/decisioning/missions/water_shoot', self.pump_callback, 10)
        self.create_subscription(Bool, 'cognition/decisioning/missions/ball_launch', self.launcher_callback, 10)

        # ========== TIMERS ================
        self.create_timer(0.02, self.send_serial_data)  # 50Hz

    def init_parameters(self):
        """Initialize node parameters."""
        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-STMicroelectronics_Barunastra_ITS_STM32_464363653030-if00')
        self.declare_parameter('baud_rate', 1000000)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

    def initialize_serial_connection(self):
        """Initialize the serial connection."""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to serial port {self.serial_port} at {self.baud_rate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.ser = None

    def pump_callback(self, msg):
        """Callback for pump subscriber."""
        self.pump_state = 1 if msg.data else 0
        self.last_pump_msg_time = self.get_clock().now()

    def launcher_callback(self, msg):
        """Callback for launcher subscriber."""
        self.launcher_state = 1 if msg.data else 0
        self.last_launcher_msg_time = self.get_clock().now()

    def timeout_check(self):
        """Check if the pump and launcher have timed out and reset if necessary."""
        if self.has_timeout_occurred(self.last_pump_msg_time, 1.0):
            self.pump_state = 0
        if self.has_timeout_occurred(self.last_launcher_msg_time, 0.2):
            self.launcher_state = 0

    def has_timeout_occurred(self, last_msg_time, timeout_seconds):
        """Check if a timeout has occurred based on the last message time."""
        current_time = self.get_clock().now()
        time_diff = current_time - last_msg_time
        return time_diff.nanoseconds > timeout_seconds * 1e9
    
    def format_serial_data(self):
        """Format the pump and launcher states as serial data."""
        pump = hex(self.pump_state)
        launcher = hex(self.launcher_state)
        return [0x69, 0x74, 0x73, int(pump, 16), int(launcher, 16)]

    def send_serial_data(self):
        """Send data to the serial port if connection exists."""
        if self.ser:
            try:
                self.timeout_check()
                data = self.format_serial_data()
                self.ser.write(bytearray(data))

                # ============= LOGGER  ================
                pump_status = "Pump ON" if self.pump_state else "Pump OFF"
                launcher_status = "Launcher ON" if self.launcher_state else "Launcher OFF"
                self.get_logger().info(pump_status)
                self.get_logger().info(launcher_status)
            except Exception as e:
                self.get_logger().error(f"Failed to send data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = STMCommunicationNode()
    rclpy.spin(node)

    # If a serial connection was established, close the serial port connection
    if node.ser:
        node.ser.close()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()