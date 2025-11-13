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
#include "ground_truth_republish/ground_truth_republish.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace ground_truth_republish {

GroundTruthRepublishNode::GroundTruthRepublishNode()
  : Node("ground_truth_republish_node"),
    have_prev_(false)
  {
    // Parameters
    source_topic_ = this->declare_parameter<std::string>("source_topic", "/ground_truth_pose");
    odom_topic_   = this->declare_parameter<std::string>("odom_topic", "/odom");
    publish_tf_   = this->declare_parameter<bool>("publish_tf", true);
    odom_frame_   = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_   = this->declare_parameter<std::string>("base_frame_id", "base_link");
    map_frame_    = this->declare_parameter<std::string>("map_frame_id", "map");
    min_dt_       = this->declare_parameter<double>("min_dt", 1e-3);   // seconds
    max_dt_       = this->declare_parameter<double>("max_dt", 0.2);    // seconds

    // Covariances: 6x6 flattened row-major
    pose_cov_ = this->declare_parameter<std::vector<double>>(
      "pose_covariance",
      std::vector<double>{
        1e-4,0,0,0,0,0,
        0,1e-4,0,0,0,0,
        0,0,1e-4,0,0,0,
        0,0,0,1e-3,0,0,
        0,0,0,0,1e-3,0,
        0,0,0,0,0,1e-3
      });

    twist_cov_ = this->declare_parameter<std::vector<double>>(
      "twist_covariance",
      std::vector<double>{
        1e-3,0,0,0,0,0,
        0,1e-3,0,0,0,0,
        0,0,1e-3,0,0,0,
        0,0,0,1e-2,0,0,
        0,0,0,0,1e-2,0,
        0,0,0,0,0,1e-2
      });

    if (pose_cov_.size() != 36 || twist_cov_.size() != 36) {
      RCLCPP_FATAL(get_logger(), "pose_covariance and twist_covariance must have 36 elements.");
      throw std::runtime_error("Bad covariance size");
    }

    // Publishers & TF
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SystemDefaultsQoS());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static map->odom (identity)
    publish_static_map_to_odom_identity();

    // Subscription (SensorDataQoS is common for stamped sensor-like inputs)
    auto qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      source_topic_, qos, std::bind(&GroundTruthRepublishNode::on_pose, this, _1));

    RCLCPP_INFO(get_logger(), "ground_truth_republish_node started: source_topic=%s odom_topic=%s publish_tf=%s",
                source_topic_.c_str(), odom_topic_.c_str(), publish_tf_ ? "true" : "false");
  }

void GroundTruthRepublishNode::publish_static_map_to_odom_identity() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf);
    RCLCPP_INFO(this->get_logger(), "Published static TF %s -> %s (identity)", map_frame_.c_str(), odom_frame_.c_str());
  }

Eigen::Quaterniond GroundTruthRepublishNode::quatFromMsg(const geometry_msgs::msg::Quaternion &q) {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

void GroundTruthRepublishNode::on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const rclcpp::Time t = msg->header.stamp;
    const auto &pmsg = msg->pose.position;
    const auto &qmsg = msg->pose.orientation;

    Eigen::Vector3d p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond q = quatFromMsg(qmsg);
    q.normalize();
    Eigen::Matrix3d R = q.toRotationMatrix();

    if (have_prev_) {
      double dt = (t - prev_t_).seconds();
      if (dt >= min_dt_ && dt <= max_dt_) {
        // Linear velocity in world
        Eigen::Vector3d v_world = (p - prev_p_) / dt;

        // Angular velocity via rotation delta (SO(3) log, approximated with AngleAxis)
        Eigen::Matrix3d Rdelta = prev_R_.transpose() * R;
        Eigen::AngleAxisd aa(Rdelta);
        Eigen::Vector3d w_world = aa.axis() * aa.angle() / dt;

        // Express in body frame at time k
        Eigen::Vector3d v_body = R.transpose() * v_world;
        Eigen::Vector3d w_body = R.transpose() * w_world;

        // Publish Odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = t;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id  = base_frame_;
        odom.pose.pose = msg->pose;

        for (size_t i = 0; i < 36; ++i) {
          odom.pose.covariance[i]  = pose_cov_[i];
          odom.twist.covariance[i] = twist_cov_[i];
        }

        odom.twist.twist.linear.x  = v_body.x();
        odom.twist.twist.linear.y  = v_body.y();
        odom.twist.twist.linear.z  = v_body.z();
        odom.twist.twist.angular.x = w_body.x();
        odom.twist.twist.angular.y = w_body.y();
        odom.twist.twist.angular.z = w_body.z();

        odom_pub_->publish(odom);

        if (publish_tf_) {
          geometry_msgs::msg::TransformStamped tf;
          tf.header.stamp = t;
          tf.header.frame_id = odom_frame_;
          tf.child_frame_id  = base_frame_;
          tf.transform.translation.x = pmsg.x;
          tf.transform.translation.y = pmsg.y;
          tf.transform.translation.z = pmsg.z;
          tf.transform.rotation = qmsg;
          tf_broadcaster_->sendTransform(tf);
        }
      } else {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 2000,
                              "Skipping twist compute: dt=%.6f out of [%.4f, %.4f]", dt, min_dt_, max_dt_);
      }
    }

    prev_t_ = t;
    prev_p_ = p;
    prev_R_ = R;
    have_prev_ = true;
  }

} //namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ground_truth_republish::GroundTruthRepublishNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


