#include "omnibot_driver/mecanum_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace omnibot_driver
{

MecanumController::MecanumController(const rclcpp::NodeOptions & options)
: Node("mecanum_controller", options),
  x_pos_(0.0),
  y_pos_(0.0),
  theta_(0.0)
{
  // Declare and get parameters with correct default values matching URDF
  this->declare_parameter("wheel_radius", 0.04);  // 4 cm radius
  this->declare_parameter("wheel_separation_width", 0.215);  // 21.5 cm between left and right wheels
  this->declare_parameter("wheel_separation_length", 0.165);  // 16.5 cm between front and back wheels
  this->declare_parameter("serial_port", "/dev/ttyACM0");
  this->declare_parameter("baud_rate", 115200);
  
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  wheel_separation_width_ = this->get_parameter("wheel_separation_width").as_double();
  wheel_separation_length_ = this->get_parameter("wheel_separation_length").as_double();
  port_name_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  
  // Create publishers and subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&MecanumController::cmdVelCallback, this, std::placeholders::_1));
    
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  
  // Create transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  // Create timer for periodic updates (100 Hz)
  update_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&MecanumController::updateCallback, this));
    
  // Setup serial communication
  if (!setupSerial()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port_name_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully connected to serial port %s", port_name_.c_str());
  }
  
  RCLCPP_INFO(this->get_logger(), "Mecanum controller initialized");
}

MecanumController::~MecanumController()
{
  // Close serial port if open
  if (serial_port_ && serial_port_->isOpen()) {
    serial_port_->close();
  }
}

bool MecanumController::setupSerial()
{
  try {
    serial_port_ = std::make_unique<serial::Serial>(
      port_name_,
      baud_rate_,
      serial::Timeout::simpleTimeout(1000));
    return serial_port_->isOpen();
  } catch (const serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    return false;
  }
}

void MecanumController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double front_left, front_right, rear_left, rear_right;
  
  // Calculate wheel velocities based on mecanum kinematics
  calculateWheelVelocities(*msg, front_left, front_right, rear_left, rear_right);
  
  // Send velocities to the controller
  sendVelocitiesToController(front_left, front_right, rear_left, rear_right);
}

void MecanumController::calculateWheelVelocities(
  const geometry_msgs::msg::Twist & twist,
  double & front_left, double & front_right,
  double & rear_left, double & rear_right)
{
  // Extract velocity components
  double vx = twist.linear.x;
  double vy = twist.linear.y;
  double omega = twist.angular.z;
  
  // Calculate wheel velocities using mecanum kinematics equations
  // For our mecanum wheel robot with 45-degree rollers:
  // Front Left (FL): +45° roller orientation
  // Front Right (FR): -45° roller orientation
  // Rear Left (RL): -45° roller orientation
  // Rear Right (RR): +45° roller orientation
  //
  // The wheel velocities are calculated as:
  // FL = (Vx - Vy - (L+W)*omega) / R
  // FR = (Vx + Vy + (L+W)*omega) / R
  // RL = (Vx + Vy - (L+W)*omega) / R
  // RR = (Vx - Vy + (L+W)*omega) / R
  // Where:
  // - L is half the wheel separation length (distance between front and back wheels)
  // - W is half the wheel separation width (distance between left and right wheels)
  // - R is the wheel radius
  // - omega is the angular velocity around the Z axis
  
  double L = wheel_separation_length_ / 2.0;  // 0.0825 m
  double W = wheel_separation_width_ / 2.0;   // 0.1075 m
  double k = L + W;  // Distance from center to wheel (for rotation component)
  
  // Calculate wheel velocities
  front_left = vx - vy - k * omega;
  front_right = vx + vy + k * omega;
  rear_left = vx + vy - k * omega;
  rear_right = vx - vy + k * omega;
  
  // Convert linear velocities to angular velocities (rad/s)
  front_left /= wheel_radius_;
  front_right /= wheel_radius_;
  rear_left /= wheel_radius_;
  rear_right /= wheel_radius_;
}

void MecanumController::sendVelocitiesToController(
  double front_left, double front_right,
  double rear_left, double rear_right)
{
  if (!serial_port_ || !serial_port_->isOpen()) {
    RCLCPP_WARN(this->get_logger(), "Serial port not open, cannot send velocities");
    return;
  }
  
  // Format: "<FL,FR,RL,RR>\n"
  // Convert rad/s to a suitable range for the microcontroller (e.g., PWM values)
  // For simplicity, we'll just send the raw rad/s values and let the STM32 handle conversion
  
  std::stringstream ss;
  ss << "<" << front_left << "," << front_right << "," << rear_left << "," << rear_right << ">\n";
  
  try {
    serial_port_->write(ss.str());
  } catch (const serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send velocities: %s", e.what());
  }
}

void MecanumController::updateCallback()
{
  // In a real implementation, we would read odometry data from the STM32 controller
  // For now, we'll just publish a dummy odometry message
  
  auto current_time = this->now();
  publishOdometry(current_time);
}

void MecanumController::publishOdometry(const rclcpp::Time & time)
{
  // Create quaternion from yaw
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  
  // Create and publish transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = time;
  transform_stamped.header.frame_id = "odom";
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform.translation.x = x_pos_;
  transform_stamped.transform.translation.y = y_pos_;
  transform_stamped.transform.translation.z = 0.0;
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();
  
  tf_broadcaster_->sendTransform(transform_stamped);
  
  // Create and publish odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  
  // Set position
  odom_msg.pose.pose.position.x = x_pos_;
  odom_msg.pose.pose.position.y = y_pos_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  
  // Publish odometry message
  odom_pub_->publish(odom_msg);
}

} // namespace omnibot_driver

// Entry point for the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<omnibot_driver::MecanumController>());
  rclcpp::shutdown();
  return 0;
}