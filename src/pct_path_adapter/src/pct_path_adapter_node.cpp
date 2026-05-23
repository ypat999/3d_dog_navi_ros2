#include "pct_path_adapter/pct_path_adapter_node.hpp"

#include <algorithm>
#include <numeric>
#include <functional>

PctPathAdapterNode::PctPathAdapterNode()
: Node("pct_path_adapter")
{
  this->declare_parameter("step_height_threshold", 0.15);
  this->declare_parameter("lookahead_distance", 3.0);
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("speed_scale_step", 0.3);
  this->declare_parameter("speed_scale_flat", 1.0);
  this->declare_parameter("step_zone_ahead", 2.0);
  this->declare_parameter("map_frame", std::string("map"));
  this->declare_parameter("robot_frame", std::string("base_link"));
  this->declare_parameter("timer_period_ms", 100);

  step_height_threshold_ = this->get_parameter("step_height_threshold").as_double();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  speed_scale_step_ = this->get_parameter("speed_scale_step").as_double();
  speed_scale_flat_ = this->get_parameter("speed_scale_flat").as_double();
  step_zone_ahead_ = this->get_parameter("step_zone_ahead").as_double();
  map_frame_ = this->get_parameter("map_frame").as_string();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  int timer_ms = this->get_parameter("timer_period_ms").as_int();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pct_path_sub_ = this->create_subscription<Path>(
    "/pct_path", 1,
    std::bind(&PctPathAdapterNode::onPctPath, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<Twist>(
    "/cmd_vel_raw", 1,
    std::bind(&PctPathAdapterNode::onCmdVel, this, std::placeholders::_1));

  local_path_pub_ = this->create_publisher<Path>("/plan", 1);
  goal_pose_pub_ = this->create_publisher<PoseStamped>("/goal_pose", 1);
  step_warning_pub_ = this->create_publisher<Bool>("/step_warning", 1);
  speed_scale_pub_ = this->create_publisher<Float32>("/speed_scale", 1);
  full_2d_path_pub_ = this->create_publisher<Path>("/pct_path_2d", 1);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_ms),
    std::bind(&PctPathAdapterNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(),
    "PctPathAdapter initialized: step_thresh=%.2f, lookahead=%.1f, "
    "goal_tol=%.2f, speed_step=%.2f, speed_flat=%.2f",
    step_height_threshold_, lookahead_distance_,
    goal_tolerance_, speed_scale_step_, speed_scale_flat_);
}

void PctPathAdapterNode::onPctPath(const Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty path");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  global_path_3d_.clear();
  global_path_3d_.reserve(msg->poses.size());
  for (const auto & ps : msg->poses) {
    PathPoint3D pt;
    pt.x = ps.pose.position.x;
    pt.y = ps.pose.position.y;
    pt.z = ps.pose.position.z;
    global_path_3d_.push_back(pt);
  }

  segments_ = splitByFloor(global_path_3d_);
  current_segment_idx_ = 0;
  current_waypoint_idx_ = 0;
  path_received_ = true;
  goal_reached_ = false;

  Path path_2d = projectTo2D(global_path_3d_);
  path_2d.header = msg->header;
  full_2d_path_pub_->publish(path_2d);

  RCLCPP_INFO(this->get_logger(),
    "Received 3D path: %zu points, %zu segments",
    global_path_3d_.size(), segments_.size());

  for (size_t i = 0; i < segments_.size(); ++i) {
    RCLCPP_INFO(this->get_logger(),
      "  Segment %zu: %zu points, has_step=%s, step_h=%.2f",
      i, segments_[i].points.size(),
      segments_[i].has_step ? "true" : "false",
      segments_[i].step_height);
  }
}

void PctPathAdapterNode::onTimer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!path_received_ || goal_reached_) {
    return;
  }

  double rx, ry, rz;
  if (!getRobotPose(rx, ry, rz)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Cannot get robot pose in %s frame", map_frame_.c_str());
    return;
  }

  if (current_segment_idx_ >= segments_.size()) {
    goal_reached_ = true;
    RCLCPP_INFO(this->get_logger(), "All segments completed");
    return;
  }

  const auto & seg = segments_[current_segment_idx_];
  const auto & pts = seg.points;

  auto closest_opt = findClosestIndex(pts, rx, ry);
  if (!closest_opt.has_value()) {
    return;
  }
  size_t closest_idx = closest_opt.value();

  double dist_to_end = std::hypot(
    rx - pts.back().x, ry - pts.back().y);
  if (dist_to_end < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(),
      "Segment %zu completed (dist=%.2f)", current_segment_idx_, dist_to_end);
    current_segment_idx_++;
    current_waypoint_idx_ = 0;

    if (current_segment_idx_ >= segments_.size()) {
      goal_reached_ = true;
      RCLCPP_INFO(this->get_logger(), "Global goal reached");
      Bool warn_msg; warn_msg.data = false;
      step_warning_pub_->publish(warn_msg);
      Float32 scale_msg; scale_msg.data = speed_scale_flat_;
      speed_scale_pub_->publish(scale_msg);
      return;
    }
    return;
  }

  double accumulated = 0.0;
  size_t lookahead_idx = closest_idx;
  for (size_t i = closest_idx + 1; i < pts.size(); ++i) {
    double dx = pts[i].x - pts[i - 1].x;
    double dy = pts[i].y - pts[i - 1].y;
    accumulated += std::hypot(dx, dy);
    if (accumulated >= lookahead_distance_) {
      lookahead_idx = i;
      break;
    }
    lookahead_idx = i;
  }

  bool near_step = false;
  double step_dz = 0.0;
  {
    double check_dist = 0.0;
    for (size_t i = closest_idx; i < pts.size() - 1; ++i) {
      double dx = pts[i + 1].x - pts[i].x;
      double dy = pts[i + 1].y - pts[i].y;
      check_dist += std::hypot(dx, dy);
      if (check_dist > step_zone_ahead_) break;

      double dz_val;
      if (detectStep(pts, i, dz_val)) {
        near_step = true;
        step_dz = std::max(step_dz, std::abs(dz_val));
      }
    }
  }

  Bool warn_msg;
  warn_msg.data = near_step;
  step_warning_pub_->publish(warn_msg);

  Float32 scale_msg;
  if (near_step) {
    double ratio = std::min(std::abs(step_dz) / 0.5, 1.0);
    scale_msg.data = static_cast<float>(
      speed_scale_flat_ - ratio * (speed_scale_flat_ - speed_scale_step_));
  } else {
    scale_msg.data = static_cast<float>(speed_scale_flat_);
  }
  speed_scale_pub_->publish(scale_msg);

  PoseStamped goal_pose;
  goal_pose.header.stamp = this->now();
  goal_pose.header.frame_id = map_frame_;
  goal_pose.pose.position.x = pts[lookahead_idx].x;
  goal_pose.pose.position.y = pts[lookahead_idx].y;
  goal_pose.pose.position.z = pts[lookahead_idx].z;

  if (lookahead_idx > closest_idx) {
    double yaw = std::atan2(
      pts[lookahead_idx].y - pts[closest_idx].y,
      pts[lookahead_idx].x - pts[closest_idx].x);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_pose.pose.orientation = tf2::toMsg(q);
  } else {
    goal_pose.pose.orientation.w = 1.0;
  }
  goal_pose_pub_->publish(goal_pose);

  Path local_path;
  local_path.header.stamp = this->now();
  local_path.header.frame_id = map_frame_;
  size_t local_end = std::min(lookahead_idx + 20, pts.size());
  for (size_t i = closest_idx; i < local_end; ++i) {
    PoseStamped ps;
    ps.header = local_path.header;
    ps.pose.position.x = pts[i].x;
    ps.pose.position.y = pts[i].y;
    ps.pose.position.z = pts[i].z;
    ps.pose.orientation.w = 1.0;
    local_path.poses.push_back(ps);
  }
  local_path_pub_->publish(local_path);
}

void PctPathAdapterNode::onCmdVel(const Twist::SharedPtr msg)
{
  (void)msg;
}

std::optional<size_t> PctPathAdapterNode::findClosestIndex(
  const std::vector<PathPoint3D> & pts, double rx, double ry) const
{
  if (pts.empty()) return std::nullopt;

  double best_dist = std::numeric_limits<double>::max();
  size_t best_idx = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    double d = std::hypot(pts[i].x - rx, pts[i].y - ry);
    if (d < best_dist) {
      best_dist = d;
      best_idx = i;
    }
  }
  return best_idx;
}

std::vector<PathSegment> PctPathAdapterNode::splitByFloor(
  const std::vector<PathPoint3D> & pts) const
{
  std::vector<PathSegment> result;
  if (pts.empty()) return result;

  PathSegment current;
  current.points.push_back(pts.front());

  for (size_t i = 1; i < pts.size(); ++i) {
    double dz = pts[i].z - pts[i - 1].z;
    double abs_dz = std::abs(dz);

    if (abs_dz > step_height_threshold_) {
      if (current.points.size() > 1) {
        current.has_step = true;
        current.step_height = dz;
        current.step_start_idx = current.points.size() - 1;
      }
      current.points.push_back(pts[i]);

      PathSegment next;
      next.points.push_back(pts[i]);
      for (size_t j = i + 1; j < pts.size(); ++j) {
        double next_dz = pts[j].z - pts[j - 1].z;
        if (std::abs(next_dz) > step_height_threshold_) {
          next.points.push_back(pts[j]);
          PathSegment after;
          after.points.push_back(pts[j]);
          for (size_t k = j + 1; k < pts.size(); ++k) {
            after.points.push_back(pts[k]);
          }
          if (after.points.size() > 1) {
            result.push_back(after);
          }
          result.push_back(next);
          result.push_back(current);
          std::reverse(result.begin(), result.end());
          return result;
        }
        next.points.push_back(pts[j]);
      }
      result.push_back(next);
      result.push_back(current);
      std::reverse(result.begin(), result.end());
      return result;
    }

    current.points.push_back(pts[i]);
  }

  result.push_back(current);
  return result;
}

double PctPathAdapterNode::computeStepHeight(
  const std::vector<PathPoint3D> & pts, size_t from, size_t to) const
{
  if (from >= to || to > pts.size()) return 0.0;
  double max_dz = 0.0;
  for (size_t i = from + 1; i < to; ++i) {
    double dz = std::abs(pts[i].z - pts[i - 1].z);
    max_dz = std::max(max_dz, dz);
  }
  return max_dz;
}

bool PctPathAdapterNode::detectStep(
  const std::vector<PathPoint3D> & pts, size_t i, double & dz) const
{
  if (i + 1 >= pts.size()) return false;
  dz = pts[i + 1].z - pts[i].z;
  return std::abs(dz) > step_height_threshold_;
}

nav_msgs::msg::Path PctPathAdapterNode::projectTo2D(
  const std::vector<PathPoint3D> & pts3d) const
{
  Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_frame_;
  path.poses.reserve(pts3d.size());
  for (const auto & pt : pts3d) {
    PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = pt.x;
    ps.pose.position.y = pt.y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  return path;
}

bool PctPathAdapterNode::getRobotPose(
  double & rx, double & ry, double & rz) const
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero);
    rx = transform.transform.translation.x;
    ry = transform.transform.translation.y;
    rz = transform.transform.translation.z;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", ex.what());
    return false;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PctPathAdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
