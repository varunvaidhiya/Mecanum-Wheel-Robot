#ifndef OMNIBOT_DRIVER_MECANUM_CONTROLLER_HPP
#define OMNIBOT_DRIVER_MECANUM_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <serial/serial.h>

namespace omnibot_driver
{

/**
 * @brief Controller for mecanum wheel robot
 * 
 * This class handles the kinematics for a 4-wheel mecanum drive robot.
 * It subscribes to velocity commands and converts them to individual
 * wheel velocities based on the mecanum wheel kinematics equations.
 */
class MecanumController : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  explicit MecanumController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~MecanumController();

private:
  // Node parameters
  double wheel_radius_; // meters
  double wheel_separation_width_; // meters (distance between left and right wheels)
  double wheel_separation_length_; // meters (distance between front and back wheels)
  
  // Serial communication
  std::unique_ptr<serial::Serial> serial_port_;
  std::string port_name_;
  int baud_rate_;
  
  // Publishers and subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr update_timer_;
  
  // Robot state
  double x_pos_;
  double y_pos_;
  double theta_;
  
  // Callback functions
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void updateCallback();
  
  // Utility functions
  bool setupSerial();
  void calculateWheelVelocities(const geometry_msgs::msg::Twist & twist, 
                               double & front_left, double & front_right,
                               double & rear_left, double & rear_right);
  void sendVelocitiesToController(double front_left, double front_right,
                                double rear_left, double rear_right);
  void publishOdometry(const rclcpp::Time & time);
};

} // namespace omnibot_driver

#endif // OMNIBOT_DRIVER_MECANUM_CONTROLLER_HPP