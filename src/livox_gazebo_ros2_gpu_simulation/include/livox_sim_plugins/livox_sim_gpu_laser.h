/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef LIVOX_SIM_PLUGINS_LIVOXSIMGPULASER_HH_
#define LIVOX_SIM_PLUGINS_LIVOXSIMGPULASER_HH_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <unordered_map>

#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/GpuRayPlugin.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sdf/sdf.hh>

namespace gazebo
{
  // 用于存储从 CSV 读取的扫描模式
  struct ScanPatternPoint
  {
    double time;
    double azimuth; // 水平角 (rad)
    double zenith;  // 垂直角 (rad)
  };

  class GazeboRosLaser : public GpuRayPlugin
  {
  public:
    GazeboRosLaser();
    ~GazeboRosLaser();
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    virtual void OnNewLaserFrame(const float *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);
                
    // 时间统计辅助函数
    void StartTiming(const std::string& method_name);
    double EndTiming(const std::string& method_name);
    void PrintTimingStats();

  protected:
    void OnScan(ConstLaserScanStampedPtr &_msg);

  private:
    void LoadThread();
    void LoadCsvPattern();

    physics::WorldPtr world_;
    sensors::GpuRaySensorPtr parent_ray_sensor_;
    rclcpp::Node::SharedPtr rosnode_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    std::string topic_name_;
    std::string frame_name_;
    std::string robot_namespace_;
    std::string world_name_;      // 世界名称
    std::string csv_file_name_;   // CSV文件路径
    int samples_per_update_;      // 每次更新处理的点数
    int downsample_;              // 降采样系数

    sdf::ElementPtr sdf;          // SDF 元素指针

    std::thread deferred_load_thread_;
    gazebo::transport::NodePtr gazebo_node_;
    gazebo::transport::SubscriberPtr laser_scan_sub_;

    // 新增: 存储扫描模式和当前索引
    std::vector<ScanPatternPoint> scan_pattern_;
    size_t scan_pattern_index_ = 0;
    
    // 时间统计相关成员变量
    std::unordered_map<std::string, double> method_time_stats_;
    std::unordered_map<std::string, int> method_call_count_;
    std::chrono::steady_clock::time_point last_print_time_;
    bool stats_initialized_ = false;
    
    // 频率控制相关成员变量
    std::chrono::steady_clock::time_point last_check_time_;
  };
}
#endif  // LIVOX_SIM_PLUGINS_LIVOXSIMGPULASER_HH_