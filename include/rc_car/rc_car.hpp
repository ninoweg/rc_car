// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <geometry_msgs/msg/twist.hpp>
// CPP
#include <memory>
#include <cmath>
#include <string>
#include <algorithm>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RCCar : public rclcpp::Node
{
public:
  RCCar() : Node("rc_car")
  {
    declare_parameter("wheelbase", 0.3);
    declare_parameter("max_linear_velocity", 1.0);
    declare_parameter("max_steering_angle", M_PI / 8);
    declare_parameter("steering_pwm_limits", std::vector<int64_t>({ 544, 1500, 2400 }));
    declare_parameter("velocity_pwm_limits", std::vector<int64_t>({ 544, 1500, 2400 }));

    get_parameter("wheelbase", wheelbase_);
    get_parameter("max_linear_velocity", max_linear_velocity_);
    get_parameter("max_steering_angle", max_steering_angle_);
    
    auto steering_pwm_limits = get_parameter("steering_pwm_limits").as_integer_array();
    if (steering_pwm_limits.size() == 3)
      steering_pwm_limits_ = steering_pwm_limits;
    else 
      RCLCPP_ERROR_STREAM(get_logger(), "steering_pwm_limits has wrong size. Required format: [min, neutral, max]");
    
    auto velocity_pwm_limits = get_parameter("velocity_pwm_limits").as_integer_array();
    if (velocity_pwm_limits.size() == 3)
      velocity_pwm_limits_ = velocity_pwm_limits;
    else 
      RCLCPP_ERROR_STREAM(get_logger(), "velocity_pwm_limits has wrong size. Required format: [min, neutral, max]");

    pub_steering_channel_ = this->create_publisher<std_msgs::msg::Int16>("channel1", 10); // steering angle 
    pub_velocity_channel_ = this->create_publisher<std_msgs::msg::Int16>("channel2", 10); // linear velocity
    
    sub_cmd_vel_ =
        this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&RCCar::cb_cmd_vel, this, _1));
    sub_cmd_vel_ackermann_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_ackermann", 10, std::bind(&RCCar::cb_cmd_vel_ackermann, this, _1));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_steering_channel_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_velocity_channel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_ackermann_;
  double wheelbase_;
  double max_linear_velocity_;
  double max_steering_angle_;
  std::vector<int64_t> steering_pwm_limits_;
  std::vector<int64_t> velocity_pwm_limits_;

  void cb_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto velocity = msg->linear.x;
    auto steering_angle{ 0.0 };
    if (abs(msg->linear.x) > 1e-3)
      steering_angle = atan((wheelbase_ * msg->angular.z) / msg->linear.x);
    else
      steering_angle = 0.0;

    pub_pwm_signals(steering_angle, velocity);
  }

  void cb_cmd_vel_ackermann(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto velocity = msg->linear.x;
    auto steering_angle = msg->angular.z;

    pub_pwm_signals(steering_angle, velocity);
  }

  void pub_pwm_signals(double steering_angle, double velocity)
  {
    auto steering_angle_ratio = std::clamp(steering_angle / max_steering_angle_, -1.0, 1.0);
    auto linear_vel_ratio = std::clamp(velocity / max_linear_velocity_, -1.0, 1.0);

    auto steering_pwm_signal = calc_pwm_signal(steering_angle_ratio, steering_pwm_limits_);
    auto velocity_pwm_signal = calc_pwm_signal(linear_vel_ratio, velocity_pwm_limits_);

    auto steering_msg = std_msgs::msg::Int16();
    steering_msg.data = steering_pwm_signal;
    pub_steering_channel_->publish(steering_msg);

    auto velocity_msg = std_msgs::msg::Int16();
    velocity_msg.data = velocity_pwm_signal;
    pub_velocity_channel_->publish(velocity_msg);
  }

  double calc_pwm_signal(double ratio, std::vector<int64_t> limits)
  {
    auto signal = limits[1];
    if (ratio > 0.0)
      signal += ratio * (limits[0] - limits[1]);
    else
      signal += ratio * (limits[1] - limits[2]);
    return signal;
  }
};