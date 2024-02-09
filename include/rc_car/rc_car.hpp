// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <geometry_msgs/msg/twist.hpp>
// CPP
#include <memory>
#include <cmath>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RCCar : public rclcpp::Node
{
public:
  RCCar()
    : Node("rc_car")
    , wheelbase_{ 0.3 }
    , max_linear_vel_{ 1.0 }
    , max_steering_angle_{ M_PI / 8 }
    , channel1_signal_limits_{ 544, 2400 }
    , channel2_signal_limits_{ 544, 2400 }
    , channel1_signal_neutral_{ 1500 }
    , channel2_signal_neutral_{ 1500 }
  {
    pub_channel_1_ = this->create_publisher<std_msgs::msg::Int16>("channel1", 10);
    pub_channel_2_ = this->create_publisher<std_msgs::msg::Int16>("channel2", 10);
    sub_cmd_vel_ =
        this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&RCCar::cb_cmd_vel, this, _1));
    sub_cmd_vel_ackermann_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_ackermann", 10, std::bind(&RCCar::cb_cmd_vel_ackermann, this, _1));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_channel_1_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_channel_2_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_ackermann_;
  double wheelbase_;
  double max_linear_vel_;
  double max_steering_angle_;
  std::pair<int, int> channel1_signal_limits_;
  std::pair<int, int> channel2_signal_limits_;
  int channel1_signal_neutral_;
  int channel2_signal_neutral_;

  void cb_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto linear_vel = msg->linear.x;
    auto steering_angle{ 0.0 };
    if (abs(msg->linear.x) > 1e-3)
      steering_angle = atan((wheelbase_ * msg->angular.z) / msg->linear.x);
    else
      steering_angle = 0.0;

    publish_pwm_signals(steering_angle, linear_vel);
  }

  void cb_cmd_vel_ackermann(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto linear_vel = msg->linear.x;
    auto steering_angle = msg->angular.z;

    publish_pwm_signals(steering_angle, linear_vel);
  }

  void publish_pwm_signals(double steering_angle, double linear_vel)
  {
    auto steering_angle_ratio = std::clamp(steering_angle / max_steering_angle_, -1.0, 1.0);
    auto linear_vel_ratio = std::clamp(linear_vel / max_linear_vel_, -1.0, 1.0);

    auto channel1_signal = calc_pwm_signal(steering_angle_ratio, channel1_signal_neutral_, channel1_signal_limits_);
    auto channel2_signal = calc_pwm_signal(linear_vel_ratio, channel2_signal_neutral_, channel2_signal_limits_);

    auto msg1 = std_msgs::msg::Int16();
    msg1.data = channel1_signal;
    pub_channel_1_->publish(msg1);

    auto msg2 = std_msgs::msg::Int16();
    msg2.data = channel2_signal;
    pub_channel_2_->publish(msg2);
  }

  double calc_pwm_signal(double ratio, double neutral, std::pair<int, int> limits)
  {
    auto signal = neutral;
    if (ratio > 0.0)
      signal += ratio * (limits.first - neutral);
    else
      signal += ratio * (neutral - limits.second);
    return signal;
  }
};