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
    , linear_vel_{ 0.0 }
    , steering_angle_{ 0.0 }
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
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_channel_1_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_channel_2_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  double linear_vel_;
  double steering_angle_;
  double wheelbase_;
  double max_linear_vel_;
  double max_steering_angle_;
  std::pair<int, int> channel1_signal_limits_;
  std::pair<int, int> channel2_signal_limits_;
  int channel1_signal_neutral_;
  int channel2_signal_neutral_;

  void cb_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_vel_ = msg->linear.x;
    if (abs(msg->linear.x) > 1e-3)
      steering_angle_ = atan((wheelbase_ * msg->angular.z) / msg->linear.x);
    else
      steering_angle_ = 0.0;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Steering angle : " << std::to_string(steering_angle_) << " rad ("
    //                                                            << std::to_string(steering_angle_ * (180.0 / M_PI))
    //                                                            << " deg)");
    // RCLCPP_INFO_STREAM(this->get_logger(), "Linear velocity: " << std::to_string(linear_vel_) << " m/s");

    auto steering_angle_ratio = std::clamp(steering_angle_ / max_steering_angle_, -1.0, 1.0);
    auto linear_vel_ratio = std::clamp(linear_vel_ / max_linear_vel_, -1.0, 1.0);
    
    auto channel1_signal = channel1_signal_neutral_;
    if (steering_angle_ratio > 0.0)
        channel1_signal = channel1_signal_neutral_ + steering_angle_ratio * (channel1_signal_limits_.first - channel1_signal_neutral_);
    else
        channel1_signal = channel1_signal_neutral_ + steering_angle_ratio * (channel1_signal_neutral_ - channel1_signal_limits_.second);
    
    auto channel2_signal = channel2_signal_neutral_;
    if (linear_vel_ratio > 0.0)
        channel2_signal = channel2_signal_neutral_ + linear_vel_ratio * (channel2_signal_limits_.first - channel2_signal_neutral_);
    else
        channel2_signal = channel2_signal_neutral_ + linear_vel_ratio * (channel2_signal_neutral_ - channel2_signal_limits_.second);

    auto msg1 = std_msgs::msg::Int16();
    msg1.data = channel1_signal; 
    pub_channel_1_->publish(msg1);

    auto msg2 = std_msgs::msg::Int16();
    msg2.data = channel2_signal; 
    pub_channel_2_->publish(msg2);
  }
};