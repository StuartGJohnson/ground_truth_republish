#ifndef GROUND_TRUTH_PUBLISH_HPP_
#define GROUND_TRUTH_PUBLISH_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace ground_truth_republish {

class GroundTruthRepublishNode : public rclcpp::Node {
public:
  GroundTruthRepublishNode();

private:
  void publish_static_map_to_odom_identity();

  static Eigen::Quaterniond quatFromMsg(const geometry_msgs::msg::Quaternion &q);

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Params
  std::string source_topic_;
  std::string odom_topic_;
  bool publish_tf_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string map_frame_;
  double min_dt_;
  double max_dt_;
  std::vector<double> pose_cov_;
  std::vector<double> twist_cov_;

  // State
  bool have_prev_;
  rclcpp::Time prev_t_;
  Eigen::Vector3d prev_p_;
  Eigen::Matrix3d prev_R_;

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

} //namespace

#endif // GROUND_TRUTH_PUBLISH_HPP_