#include <cstdio>
#include "rc_car/rc_car.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RCCar>());
  rclcpp::shutdown();
  return 0;
}
