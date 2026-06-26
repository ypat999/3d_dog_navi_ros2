#pragma once

#include <vector>
#include <mutex>
#include <optional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

struct PathPoint3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PathSegment
{
  std::vector<PathPoint3D> points;
  bool has_step = false;
  double step_height = 0.0;
  size_t step_start_idx = 0;
};

class PctPathAdapterNode : public rclcpp::Node
{
public:
  using Path = nav_msgs::msg::Path;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::Twist;
  using Bool = std_msgs::msg::Bool;
  using Float32 = std_msgs::msg::Float32;

  PctPathAdapterNode();
  ~PctPathAdapterNode() = default;

private:
  void onPctPath(const Path::SharedPtr msg);
  void onTimer();
  void onCmdVel(const Twist::SharedPtr msg);

  std::optional<size_t> findClosestIndex(const std::vector<PathPoint3D> & pts,
                                         double rx, double ry) const;

  std::vector<PathSegment> splitByFloor(const std::vector<PathPoint3D> & pts) const;

  double computeStepHeight(const std::vector<PathPoint3D> & pts,
                           size_t from, size_t to) const;

  bool detectStep(const std::vector<PathPoint3D> & pts,
                  size_t i, double & dz) const;

  Path projectTo2D(const std::vector<PathPoint3D> & pts3d) const;

  bool getRobotPose(double & rx, double & ry, double & rz) const;

  rclcpp::Subscription<Path>::SharedPtr pct_path_sub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<Bool>::SharedPtr step_warning_pub_;
  rclcpp::Publisher<Float32>::SharedPtr speed_scale_pub_;
  rclcpp::Publisher<Path>::SharedPtr full_2d_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  std::vector<PathPoint3D> global_path_3d_;
  std::vector<PathSegment> segments_;
  size_t current_segment_idx_ = 0;
  size_t current_waypoint_idx_ = 0;
  bool path_received_ = false;
  bool goal_reached_ = true;

  double step_height_threshold_;
  double lookahead_distance_;
  double goal_tolerance_;
  double speed_scale_step_;
  double speed_scale_flat_;
  double step_zone_ahead_;
  std::string map_frame_;
  std::string robot_frame_;
};
