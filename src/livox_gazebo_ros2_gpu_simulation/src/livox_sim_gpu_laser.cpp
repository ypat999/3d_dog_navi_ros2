/*
 * Copyright 2013 Open Source Robotics Foundation
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

/*
   Desc: GazeboRosGpuLaser plugin for simulating ray sensors in Gazebo
   Author: Mihai Emanuel Dolha
   Date: 29 March 2012
 */
 #include <algorithm>
 #include <string>
 #include <vector>
 #include <assert.h>
 #include <fstream>
 #include <sstream>
 #include <iomanip>
 
 #include <sdf/sdf.hh>
 #include <ament_index_cpp/get_package_share_directory.hpp>
 
 #include <gazebo/physics/World.hh>
 #include <gazebo/sensors/Sensor.hh>
 #include <gazebo/common/Exception.hh>
 #include <gazebo/sensors/GpuRaySensor.hh>
 #include <gazebo/sensors/SensorTypes.hh>
 #include <gazebo/transport/transport.hh>
 
 // PointCloud2 message helper
 #include "sensor_msgs/point_cloud2_iterator.hpp"
 
 #include "livox_sim_plugins/livox_sim_gpu_laser.h"
 #include <ignition/math/Rand.hh>
 
 namespace gazebo
 {
 GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLaser)
 
 GazeboRosLaser::GazeboRosLaser() {
  // 初始化时间统计
  last_print_time_ = std::chrono::steady_clock::now();
  last_check_time_ = std::chrono::steady_clock::now();
  stats_initialized_ = true;
}
 
 // 时间统计辅助函数
void GazeboRosLaser::StartTiming(const std::string& method_name) {
  auto now = std::chrono::steady_clock::now();
  method_time_stats_[method_name] = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
}

double GazeboRosLaser::EndTiming(const std::string& method_name) {
  auto now = std::chrono::steady_clock::now();
  double start_time = method_time_stats_[method_name];
  double end_time = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
  double elapsed_ms = (end_time - start_time) / 1000.0; // 转换为毫秒
  
  // 累加时间统计
  if (method_time_stats_.find(method_name + "_total") == method_time_stats_.end()) {
    method_time_stats_[method_name + "_total"] = 0.0;
    method_call_count_[method_name] = 0;
  }
  
  method_time_stats_[method_name + "_total"] += elapsed_ms;
  method_call_count_[method_name]++;
  
  return elapsed_ms;
}

void GazeboRosLaser::PrintTimingStats() {
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time_).count();
  
  // 每60秒打印一次统计信息
  if (duration >= 60) {
    RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "===== GPU Laser Timing Statistics =====");
    
    for (const auto& entry : method_call_count_) {
      const std::string& method_name = entry.first;
      int call_count = entry.second;
      double total_time = method_time_stats_[method_name + "_total"];
      double avg_time = call_count > 0 ? total_time / call_count : 0.0;
      
      RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), 
                  "Method: %-20s | Calls: %6d | Total: %8.2f ms | Avg: %6.2f ms",
                  method_name.c_str(), call_count, total_time, avg_time);
    }
    
    last_print_time_ = now;
  }
}

GazeboRosLaser::~GazeboRosLaser()
 {
   RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Shutting down GPU Laser");
   if (this->rosnode_) {
     this->rosnode_.reset();
   }
   RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Unloaded");
 }
 
void GazeboRosLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
 RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Loading GazeboRosLaser plugin..."); 
 // Save SDF pointer
 this->sdf = _sdf;
 GpuRayPlugin::Load(_parent, this->sdf);
 this->world_name_ = _parent->WorldName();
 this->world_ = physics::get_world(this->world_name_);
  this->parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosLaser controller requires a GpuRaySensor as its parent");

  // Read necessary parameters
  this->robot_namespace_ = _sdf->Get<std::string>("robotNamespace", "").first;
  this->frame_name_ = _sdf->Get<std::string>("frameName", "world").first;
  this->topic_name_ = _sdf->Get<std::string>("topic", "/livox/points").first;

  this->csv_file_name_ = _sdf->Get<std::string>("csv_file_name", "").first;
  this->samples_per_update_ = _sdf->Get<int>("samples", 1000).first;
  this->downsample_ = _sdf->Get<int>("downsample", 1).first;
  if (this->downsample_ < 1) this->downsample_ = 1;

  if (this->csv_file_name_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("gpu_laser"), "CSV file name is empty!");
    return;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Plugin parameters loaded:");
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Robot namespace: %s", this->robot_namespace_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Frame name: %s", this->frame_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Topic name: %s", this->topic_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - CSV file: %s", this->csv_file_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Samples per update: %d", this->samples_per_update_);
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Downsample factor: %d", this->downsample_);
 
   if (!rclcpp::ok())
   {
     RCLCPP_FATAL(rclcpp::get_logger("gpu_laser"), "A ROS node for Gazebo has not been initialized...");
     return;
   }
 
   this->deferred_load_thread_ = std::thread(std::bind(&GazeboRosLaser::LoadThread, this));
 }
 
void GazeboRosLaser::LoadThread()
{
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Initializing Livox GPU Laser plugin...");
  
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Gazebo node initialized for world: %s", this->world_name_.c_str());

  this->rosnode_ = rclcpp::Node::make_shared("livox_sim_gpu_laser");
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "ROS2 node created: livox_sim_gpu_laser");

  this->pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::PointCloud2>(this->topic_name_, 10);
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "PointCloud2 publisher created on topic: %s", this->topic_name_.c_str());
  
  this->LoadCsvPattern();

  this->laser_scan_sub_ = this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &GazeboRosLaser::OnScan, this);
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Subscribed to Gazebo laser topic: %s", this->parent_ray_sensor_->Topic().c_str());

  this->parent_ray_sensor_->SetActive(true);
  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Parent ray sensor activated");

  RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Livox GPU Laser Plugin fully loaded and ready.");
}

void GazeboRosLaser::OnNewLaserFrame(const float */*_image*/,
                unsigned int /*_width*/, unsigned int /*_height*/,
                unsigned int /*_depth*/, const std::string &/*_format*/)
{
  // This method is required by GpuRayPlugin but we use OnScan instead
  // Keep empty implementation to satisfy the interface
}

// Helper function to resolve package:// URIs
std::string ResolvePackageURI(const std::string& uri)
{
    if (uri.find("package://") == 0) {
        std::string path = uri.substr(10);
        size_t slash_pos = path.find('/');
        if (slash_pos != std::string::npos) {
            std::string package_name = path.substr(0, slash_pos);
            std::string relative_path = path.substr(slash_pos + 1);
            
            try {
                std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
                return package_path + "/" + relative_path;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("gpu_laser"), 
                    "Failed to resolve package:// URI '%s': %s", uri.c_str(), e.what());
                return uri;
            }
        }
    }
    return uri;
}
 
void GazeboRosLaser::LoadCsvPattern()
{
    RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Loading CSV pattern from: %s", this->csv_file_name_.c_str());
    
    std::string resolved_path = ResolvePackageURI(this->csv_file_name_);
    RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "Resolved path: %s", resolved_path.c_str());
    
    std::ifstream file(resolved_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("gpu_laser"), "Could not open CSV file: %s", resolved_path.c_str());
        return;
    }

    std::string line;
    // Skip header
    std::getline(file, line); 
    RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "CSV header: %s", line.c_str());

    int line_count = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        ScanPatternPoint point;
        
        // Time/s
        std::getline(ss, value, ',');
        point.time = std::stod(value);

        // Azimuth/deg -> to rad
        std::getline(ss, value, ',');
        point.azimuth = std::stod(value) * M_PI / 180.0;

        // Zenith/deg -> to rad
        std::getline(ss, value, ',');
        point.zenith = (90 - std::stod(value)) * M_PI / 180.0;
        
        this->scan_pattern_.push_back(point);
        line_count++;
        
        // 打印前几个点的信息用于调试
        if (line_count <= 3) {
            RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), 
                "Point %d: azimuth=%.4f rad, zenith=%.4f rad", 
                line_count, point.azimuth, point.zenith);
        }
    }
    file.close();
    RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), 
        "Successfully loaded %zu scan pattern points from CSV", this->scan_pattern_.size());
}
 
void GazeboRosLaser::OnScan(ConstLaserScanStampedPtr &_msg)
{
    // 第一步：快速检查是否需要处理
    if (!_msg) {
        RCLCPP_ERROR(rclcpp::get_logger("gpu_laser"), "Received null laser scan message!");
        return;
    }

    if (this->scan_pattern_.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("gpu_laser"), "Scan pattern is empty, skipping OnScan");
        return;
    }

    if (!this->parent_ray_sensor_) {
        RCLCPP_ERROR(rclcpp::get_logger("gpu_laser"), "Parent ray sensor is null!");
        return;
    }

    StartTiming("OnScan");

    // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("gpu_laser"), 
    //     *rclcpp::Clock::make_shared(), 1000, 
    //     "OnScan called, pattern index: %zu/%zu", 
    //     this->scan_pattern_index_, this->scan_pattern_.size());

    try {
    // 1. Create PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = rclcpp::Time(_msg->time().sec(), _msg->time().nsec());
    cloud_msg.header.frame_id = this->frame_name_;
    cloud_msg.height = 1;
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 16; // 4 fields * 4 bytes each
 
     sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
     modifier.setPointCloud2Fields(4,
         "x", 1, sensor_msgs::msg::PointField::FLOAT32,
         "y", 1, sensor_msgs::msg::PointField::FLOAT32,
         "z", 1, sensor_msgs::msg::PointField::FLOAT32,
         "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
 
    // 2. Process samples_per_update_ points in the CSV loop
    const int points_to_process = this->samples_per_update_;
    const size_t pattern_size = this->scan_pattern_.size();
    
    // Use a temporary container to collect valid points, to avoid iterator invalidation issues
    struct Point3D {
        float x, y, z;
        float intensity; // 添加intensity字段
    };
    std::vector<Point3D> valid_points;
    valid_points.reserve(points_to_process / this->downsample_);
 
    // Get the angle and size information from the original sensor
    const double min_horz_angle = this->parent_ray_sensor_->AngleMin().Radian();
    const double max_horz_angle = this->parent_ray_sensor_->AngleMax().Radian();
    const double horz_angle_res = this->parent_ray_sensor_->AngleResolution();
    const double min_vert_angle = this->parent_ray_sensor_->VerticalAngleMin().Radian();
    const double max_vert_angle = this->parent_ray_sensor_->VerticalAngleMax().Radian();
    const double vert_angle_res = this->parent_ray_sensor_->VerticalAngleResolution();
    const int ray_count = this->parent_ray_sensor_->RayCount();
    const int vertical_ray_count = this->parent_ray_sensor_->VerticalRayCount();
    
    // Check if the angle resolution is valid
    if (horz_angle_res <= 0 || vert_angle_res <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("gpu_laser"), 
            "Invalid angle resolution: horz=%.6f, vert=%.6f", horz_angle_res, vert_angle_res);
        return;
    }
    
    // Only print the sensor parameters when the first call is made
    static bool first_call = true;
    if (first_call) {
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "=== Sensor parameters ===");
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - CSV pattern size: %zu points", pattern_size);
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Samples per update: %d", points_to_process);
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Horizontal angle: [%.4f, %.4f] rad ([%.1f, %.1f] deg), resolution: %.6f", 
            min_horz_angle, max_horz_angle, min_horz_angle*180/M_PI, max_horz_angle*180/M_PI, horz_angle_res);
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Vertical angle: [%.4f, %.4f] rad ([%.1f, %.1f] deg), resolution: %.6f", 
            min_vert_angle, max_vert_angle, min_vert_angle*180/M_PI, max_vert_angle*180/M_PI, vert_angle_res);
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Ray count: %d x %d = %d total", 
            ray_count, vertical_ray_count, ray_count * vertical_ray_count);
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Range: [%.2f, %.2f] m", 
            this->parent_ray_sensor_->RangeMin(), this->parent_ray_sensor_->RangeMax());
        
        // 打印CSV角度范围
        double min_csv_azimuth = 1e10, max_csv_azimuth = -1e10;
        double min_csv_zenith = 1e10, max_csv_zenith = -1e10;
        for (const auto& pt : this->scan_pattern_) {
            min_csv_azimuth = std::min(min_csv_azimuth, pt.azimuth);
            max_csv_azimuth = std::max(max_csv_azimuth, pt.azimuth);
            min_csv_zenith = std::min(min_csv_zenith, pt.zenith);
            max_csv_zenith = std::max(max_csv_zenith, pt.zenith);
        }
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "=== CSV angle range ===");
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Azimuth: [%.4f, %.4f] rad ([%.1f, %.1f] deg)", 
            min_csv_azimuth, max_csv_azimuth, min_csv_azimuth*180/M_PI, max_csv_azimuth*180/M_PI);
        RCLCPP_INFO(rclcpp::get_logger("gpu_laser"), "  - Zenith: [%.4f, %.4f] rad ([%.1f, %.1f] deg)", 
            min_csv_zenith, max_csv_zenith, min_csv_zenith*180/M_PI, max_csv_zenith*180/M_PI);
        
        first_call = false;
    }
    // return;

    int invalid_range_count = 0;
    int out_of_bounds_count = 0;  // Points out of bounds
    int invalid_distance_count = 0;  // Invalid distance points
    
    // 从当前索引开始，处理samples_per_update_个点，在CSV中循环
    // #pragma omp parallel for
    for (int i = 0; i < points_to_process; ++i) {
        size_t current_index = (this->scan_pattern_index_ + i) % pattern_size;
        const auto& target_point = this->scan_pattern_[current_index];

        // 首先检查角度是否在传感器视场范围内
        if (target_point.azimuth < min_horz_angle || target_point.azimuth > max_horz_angle ||
            target_point.zenith < min_vert_angle || target_point.zenith > max_vert_angle) {
            invalid_range_count++;
            out_of_bounds_count++;
            continue;
        }

        // 3. 将目标角度转换为深度图的像素索引 (u, v)
        int u = static_cast<int>(std::round((target_point.azimuth - min_horz_angle) / horz_angle_res));
        int v = static_cast<int>(std::round((target_point.zenith - min_vert_angle) / vert_angle_res));
        
        // 再次边界检查（防止舍入误差导致的越界）
        if (u < 0 || u >= ray_count || v < 0 || v >= vertical_ray_count) {
            invalid_range_count++;
            out_of_bounds_count++;
            continue;
        }

        // 4. Get range from depth map directly
        int index = v * ray_count + u;
        double range = _msg->scan().ranges(index);
        // 获取强度信息
        float intensity = 0.0;
        if (_msg->scan().intensities_size() > index) {
            intensity = _msg->scan().intensities(index);
        }

        if (range < this->parent_ray_sensor_->RangeMax() && std::isfinite(range)) {
            // 5. Convert spherical coordinates to Cartesian coordinates
            double x = range * cos(target_point.zenith) * cos(target_point.azimuth);
            double y = range * cos(target_point.zenith) * sin(target_point.azimuth);
            double z = range * sin(target_point.zenith);

            // Add to valid points container
            valid_points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), intensity});
        } else {
            invalid_range_count++;
            invalid_distance_count++;
        }
    }
    // return;

    // RCLCPP_INFO(rclcpp::get_logger("gpu_laser"),
    //     "Point processing complete. Valid points: %zu, Invalid ranges: %d",
    //     valid_points.size(), invalid_range_count);

    // If the number of invalid points is too high, emit a warning
    if (invalid_range_count > points_to_process / 2) {
        RCLCPP_WARN(rclcpp::get_logger("gpu_laser"), 
            "High number of invalid ranges: %d/%d (%.1f%%) - Out of bounds: %d, Invalid distance: %d", 
            invalid_range_count, points_to_process, 100.0 * invalid_range_count / points_to_process,
            out_of_bounds_count, invalid_distance_count);
    }
    // return;

    // Set the point cloud size and fill the data
    cloud_msg.width = valid_points.size();
    modifier.resize(valid_points.size());
    
    if (!valid_points.empty()) {
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
        
        for (const auto& pt : valid_points) {
            *iter_x = pt.x;
            *iter_y = pt.y;
            *iter_z = pt.z;
            *iter_intensity = pt.intensity;
            ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("gpu_laser"), 
        "Publishing point cloud: %zu valid points (processed %d/%zu)", 
        valid_points.size(), points_to_process, pattern_size);

    // 6. Publish the point cloud
    this->pub_->publish(cloud_msg);

    // 7. Update the starting index of the next scan, in the CSV loop
    this->scan_pattern_index_ = (this->scan_pattern_index_ + points_to_process) % pattern_size;
    
    RCLCPP_DEBUG(rclcpp::get_logger("gpu_laser"), 
        "Next scan will start from CSV index: %zu/%zu", this->scan_pattern_index_, pattern_size);
    
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("gpu_laser"), "Exception in OnScan: %s", e.what());
        return;
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("gpu_laser"), "Unknown exception in OnScan");
        return;
    }
    
    double elapsed = EndTiming("OnScan");
    RCLCPP_DEBUG(rclcpp::get_logger("gpu_laser"), "OnScan method took %.2f ms", elapsed);
    
    // 定期打印统计信息
    PrintTimingStats();
}

 }