// Copyright (c) 2026 Peppermint Robotics. All rights reserved.
//
// cobra_driver_node: Serial bridge between ROS2 and the Waveshare Cobra Flex
// ESP32-S3 chassis. Converts cmd_vel to JSON motor commands, parses feedback
// JSON to produce odometry, and broadcasts the odom→base_link TF.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "cobra_driver/serial_port.hpp"

using namespace std::chrono_literals;

namespace cobra_driver
{

class CobraDriverNode : public rclcpp::Node
{
public:
  CobraDriverNode()
  : Node("cobra_driver_node")
  {
    // --------------- Parameter declarations ---------------
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("wheel_diameter", 0.0739);
    this->declare_parameter<double>("track_width", 0.159);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<int>("cmd_vel_timeout_ms", 2000);

    serial_port_name_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
    track_width_ = this->get_parameter("track_width").as_double();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    cmd_vel_timeout_ms_ = this->get_parameter("cmd_vel_timeout_ms").as_int();

    // --------------- Publishers ---------------
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("/battery_voltage", 10);

    // --------------- TF broadcaster ---------------
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // --------------- Subscriber ---------------
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CobraDriverNode::cmdVelCallback, this, std::placeholders::_1));

    // --------------- Open serial port ---------------
    if (!serial_.open(serial_port_name_, baud_rate_)) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to open serial port %s at %d baud",
        serial_port_name_.c_str(), baud_rate_);
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "Opened serial port %s at %d baud",
      serial_port_name_.c_str(), baud_rate_);

    // Enable continuous feedback stream from the ESP32 (T:131, cmd:1)
    serial_.write("{\"T\":131,\"cmd\":1}\n");
    RCLCPP_INFO(this->get_logger(), "Enabled ESP32 feedback stream");

    // --------------- Serial read thread ---------------
    // Runs in a separate thread to avoid blocking the ROS2 executor.
    serial_thread_running_ = true;
    serial_thread_ = std::thread(&CobraDriverNode::serialReadLoop, this);

    // --------------- Keepalive timer ---------------
    // If no cmd_vel arrives for cmd_vel_timeout_ms, send zero velocity to
    // keep the ESP32 heartbeat alive (motors auto-stop after 3s without a command).
    keepalive_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(cmd_vel_timeout_ms_ / 2),
      std::bind(&CobraDriverNode::keepaliveCallback, this));

    RCLCPP_INFO(this->get_logger(), "cobra_driver_node initialised");
  }

  ~CobraDriverNode() override
  {
    serial_thread_running_ = false;
    if (serial_thread_.joinable()) {
      serial_thread_.join();
    }
    // Send emergency stop before closing
    if (serial_.isOpen()) {
      serial_.write("{\"T\":0}\n");
      serial_.close();
    }
  }

private:
  // =====================================================================
  //  cmd_vel → serial JSON
  // =====================================================================
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(serial_write_mutex_);
    last_cmd_vel_time_ = this->now();

    // Build the JSON velocity command: {"T":13,"X":<linear.x>,"Z":<angular.z>}
    std::string cmd = "{\"T\":13,\"X\":" +
      std::to_string(msg->linear.x) + ",\"Z\":" +
      std::to_string(msg->angular.z) + "}\n";
    serial_.write(cmd);
  }

  // =====================================================================
  //  Keepalive — prevent the 3-second watchdog from triggering
  // =====================================================================
  void keepaliveCallback()
  {
    std::lock_guard<std::mutex> lock(serial_write_mutex_);
    auto elapsed = (this->now() - last_cmd_vel_time_).nanoseconds() / 1e6;
    if (elapsed >= cmd_vel_timeout_ms_) {
      // No cmd_vel for too long — send zero velocity keepalive
      serial_.write("{\"T\":13,\"X\":0,\"Z\":0}\n");
    }
  }

  // =====================================================================
  //  Serial read thread — parses ESP32 JSON feedback
  // =====================================================================
  void serialReadLoop()
  {
    while (serial_thread_running_ && rclcpp::ok()) {
      std::string line = serial_.readLine(100);  // 100ms timeout
      if (line.empty()) {
        continue;
      }
      parseFeedback(line);
    }
  }

  // =====================================================================
  //  JSON feedback parser
  //  Expected format: {"T":1001,"M1":0,"M2":0,"M3":0,"M4":0,"odl":0,"odr":0,"v":1110}
  //
  //  We do minimal JSON parsing (no library) because the firmware output
  //  format is fixed and simple. We extract fields by searching for keys.
  // =====================================================================
  void parseFeedback(const std::string & line)
  {
    // Only process T:1001 base-info messages
    if (line.find("\"T\":1001") == std::string::npos) {
      return;
    }

    double odl = extractJsonDouble(line, "\"odl\":");
    double odr = extractJsonDouble(line, "\"odr\":");
    double v = extractJsonDouble(line, "\"v\":");

    // Publish battery voltage (v field is voltage × 100)
    auto battery_msg = std_msgs::msg::Float32();
    battery_msg.data = static_cast<float>(v / 100.0);
    battery_pub_->publish(battery_msg);

    // Compute odometry from cumulative odl/odr (in centimetres)
    updateOdometry(odl, odr);
  }

  /// Extract a numeric value following the given key in a JSON-like string.
  /// E.g., extractJsonDouble("{\"odl\":123}", "\"odl\":") → 123.0
  static double extractJsonDouble(const std::string & json, const std::string & key)
  {
    auto pos = json.find(key);
    if (pos == std::string::npos) {
      return 0.0;
    }
    pos += key.size();
    // Skip optional whitespace
    while (pos < json.size() && json[pos] == ' ') {
      ++pos;
    }
    try {
      return std::stod(json.substr(pos));
    } catch (...) {
      return 0.0;
    }
  }

  // =====================================================================
  //  Odometry computation — differential drive kinematics
  //
  //  odl/odr are cumulative values in centimetres from the ESP32.
  //  We track the delta between consecutive readings.
  //
  //  delta_left_m  = (current_odl - prev_odl) * 0.01
  //  delta_right_m = (current_odr - prev_odr) * 0.01
  //  delta_s       = (delta_right + delta_left) / 2.0
  //  delta_theta   = (delta_right - delta_left) / track_width
  //  x += delta_s * cos(theta + delta_theta/2)
  //  y += delta_s * sin(theta + delta_theta/2)
  //  theta += delta_theta
  // =====================================================================
  void updateOdometry(double odl_cm, double odr_cm)
  {
    auto now = this->now();

    if (!odom_initialized_) {
      // First reading — just store the baseline values
      prev_odl_cm_ = odl_cm;
      prev_odr_cm_ = odr_cm;
      prev_odom_time_ = now;
      odom_initialized_ = true;
      return;
    }

    // Compute deltas in metres
    double delta_left_m = (odl_cm - prev_odl_cm_) * 0.01;
    double delta_right_m = (odr_cm - prev_odr_cm_) * 0.01;

    prev_odl_cm_ = odl_cm;
    prev_odr_cm_ = odr_cm;

    double delta_s = (delta_right_m + delta_left_m) / 2.0;
    double delta_theta = (delta_right_m - delta_left_m) / track_width_;

    // Update pose using mid-point integration
    double mid_theta = theta_ + delta_theta / 2.0;
    x_ += delta_s * std::cos(mid_theta);
    y_ += delta_s * std::sin(mid_theta);
    theta_ += delta_theta;

    // Normalise theta to [-PI, PI]
    while (theta_ > M_PI) { theta_ -= 2.0 * M_PI; }
    while (theta_ < -M_PI) { theta_ += 2.0 * M_PI; }

    // Compute velocities
    double dt = (now - prev_odom_time_).seconds();
    double vx = 0.0;
    double vth = 0.0;
    if (dt > 0.0) {
      vx = delta_s / dt;
      vth = delta_theta / dt;
    }
    prev_odom_time_ = now;

    // Build quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    // ---- Publish TF: odom → base_link ----
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = now;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf_msg);
    }

    // ---- Publish Odometry message ----
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Pose
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Pose covariance (6x6, row-major) — reasonable defaults for wheel odometry
    // Diagonal: [x, y, z, roll, pitch, yaw]
    odom_msg.pose.covariance[0] = 0.01;    // x
    odom_msg.pose.covariance[7] = 0.01;    // y
    odom_msg.pose.covariance[14] = 1e6;    // z (not measured)
    odom_msg.pose.covariance[21] = 1e6;    // roll (not measured)
    odom_msg.pose.covariance[28] = 1e6;    // pitch (not measured)
    odom_msg.pose.covariance[35] = 0.03;   // yaw

    // Twist
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.angular.z = vth;

    // Twist covariance
    odom_msg.twist.covariance[0] = 0.01;   // linear x
    odom_msg.twist.covariance[7] = 0.01;   // linear y
    odom_msg.twist.covariance[14] = 1e6;   // linear z
    odom_msg.twist.covariance[21] = 1e6;   // angular x
    odom_msg.twist.covariance[28] = 1e6;   // angular y
    odom_msg.twist.covariance[35] = 0.03;  // angular z

    odom_pub_->publish(odom_msg);
  }

  // =====================================================================
  //  Member variables
  // =====================================================================

  // Serial
  SerialPort serial_;
  std::mutex serial_write_mutex_;
  std::string serial_port_name_;
  int baud_rate_{115200};

  // Serial read thread
  std::thread serial_thread_;
  std::atomic<bool> serial_thread_running_{false};

  // Robot parameters
  double wheel_diameter_{0.0739};
  double track_width_{0.159};

  // Frame IDs
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  bool publish_tf_{true};

  // Keepalive
  int cmd_vel_timeout_ms_{2000};
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr keepalive_timer_;

  // Odometry state
  bool odom_initialized_{false};
  double prev_odl_cm_{0.0};
  double prev_odr_cm_{0.0};
  rclcpp::Time prev_odom_time_{0, 0, RCL_ROS_TIME};
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace cobra_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cobra_driver::CobraDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
