/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>

#include "rviz-3d-nav-goal-tool/goal_tool.hpp"

namespace rviz_3d_nav_goal_tool
{

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty( "Topic", "goal",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName( "3D Nav Goal" );
  
  // Create ROS2 node
  node_ = std::make_shared<rclcpp::Node>("goal_3d_tool_node");
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  if (node_) {
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      topic_property_->getStdString(), rclcpp::QoS(1));
  }
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  RCLCPP_WARN(node_->get_logger(), "3D Goal Set");
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  
  // Create quaternion from yaw angle
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  
  // Create PoseStamped message
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->now();
  goal.header.frame_id = fixed_frame;
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.pose.orientation = tf2::toMsg(quat);
  
  RCLCPP_INFO(node_->get_logger(), 
              "Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f", 
              fixed_frame.c_str(),
              goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
              goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
  
  if (pub_) {
    pub_->publish(goal);
  }
}

} // end namespace rviz_3d_nav_goal_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_3d_nav_goal_tool::Goal3DTool, rviz_common::Tool)
