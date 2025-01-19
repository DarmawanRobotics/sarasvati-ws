#include <rclcpp/rclcpp.hpp>
#include <libserialport.h>
#include <lwgps/lwgps.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

using namespace std::chrono_literals;

class F9PIONode : public rclcpp::Node {
public:
  F9PIONode() : Node("f9p_io_node") {
    initializeParameters();
    gps_timer = this->create_wall_timer(10ms, std::bind(&F9PIONode::gpsUpdateCallback, this));
    gps_publisher = this->create_publisher<sensor_msgs::msg::NavSatFix>("sasv/io/gps", 10);

    if (!initializeGps()) {
      RCLCPP_ERROR(this->get_logger(), "GPS initialization failed");
      rclcpp::shutdown();
    }
  }

  void gpsUpdateCallback() {
    if (!processGpsData()) {
      RCLCPP_ERROR(this->get_logger(), "GPS data processing failed");
      rclcpp::shutdown();
    }
  }

private:
  std::string gps_port;
  int gps_baud_rate;
  rclcpp::TimerBase::SharedPtr gps_timer;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher;
  struct sp_port *serial_port = nullptr;
  lwgps_t gps;

  void initializeParameters() {
    this->declare_parameter<std::string>("gps.port", "/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0");
    this->declare_parameter<int>("gps.baud_rate", 115200);

    this->get_parameter("gps.port", gps_port);
    this->get_parameter("gps.baud_rate", gps_baud_rate);

    RCLCPP_INFO(this->get_logger(), "GPS Port: %s", gps_port.c_str());
    RCLCPP_INFO(this->get_logger(), "GPS Baud Rate: %d", gps_baud_rate);
  }

  bool initializeGps() {
    if (sp_get_port_by_name(gps_port.c_str(), &serial_port) != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Serial port not found: %s", gps_port.c_str());
      return false;
    }

    sp_close(serial_port);
    if (sp_open(serial_port, SP_MODE_READ_WRITE) != SP_OK ||
        sp_set_baudrate(serial_port, gps_baud_rate) != SP_OK ||
        sp_set_bits(serial_port, 8) != SP_OK ||
        sp_set_parity(serial_port, SP_PARITY_NONE) != SP_OK ||
        sp_set_stopbits(serial_port, 1) != SP_OK ||
        sp_set_flowcontrol(serial_port, SP_FLOWCONTROL_NONE) != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure serial port");
      return false;
    }
    return true;
  }

  bool processGpsData() {
    uint8_t serial_data = 0;
    uint8_t gps_data_processed = 0;

    sp_return serial_return;
    while ((serial_return = sp_nonblocking_read(serial_port, &serial_data, 1)) > 0) {
      gps_data_processed += lwgps_process(&gps, &serial_data, 1);
    }

    if (serial_return < 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial port read failed");
      return false;
    }

    if (gps_data_processed < 1) {
      return true;
    }

    publishGpsData();
    return true;
  }

  void publishGpsData() {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps";

    msg.status.status = (gps.fix == 0 || gps.fix_mode == 1)
                        ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX
                        : sensor_msgs::msg::NavSatStatus::STATUS_FIX;

    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                         sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;

    msg.latitude = gps.latitude;
    msg.longitude = gps.longitude;
    msg.altitude = 0.0;
    gps_publisher->publish(msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<F9PIONode>());
  rclcpp::shutdown();
  return 0;
}
