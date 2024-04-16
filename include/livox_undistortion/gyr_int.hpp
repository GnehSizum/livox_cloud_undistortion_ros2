#ifndef GYR_INT_H
#define GYR_INT_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "sophus/so3.hpp"

inline double get_time_sec(const builtin_interfaces::msg::Time &time) {
  return rclcpp::Time(time).seconds();
}

inline rclcpp::Time get_ros_time(double timestamp) {
  int32_t sec = std::floor(timestamp);
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  uint32_t nanosec = nanosec_d;
  return rclcpp::Time(sec, nanosec);
}

class GyrInt {
 public:
  GyrInt();
  void Integrate(const sensor_msgs::msg::Imu::ConstPtr &imu);
  void Reset(double start_timestamp,
             const sensor_msgs::msg::Imu::ConstPtr &lastimu);

  const Sophus::SO3d GetRot() const;

 private:
  // Sophus::SO3d r_;
  /// last_imu_ is
  double start_timestamp_;
  sensor_msgs::msg::Imu::ConstPtr last_imu_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::vector<sensor_msgs::msg::Imu::ConstPtr> v_imu_;
  std::vector<Sophus::SO3d> v_rot_;
};

#endif