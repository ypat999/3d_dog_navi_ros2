//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#define SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H

#include <gazebo/plugins/RayPlugin.hh>
#include "livox_ode_multiray_shape.h"
#include <gazebo_ros/node.hpp>
#include <gazebo/gazebo.hh>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace gazebo
{
   struct AviaRotateInfo
   {
      double time;
      double azimuth;
      double zenith;
   };


   class LivoxPointsPlugin : public RayPlugin
   {
   public:
      LivoxPointsPlugin();

      virtual ~LivoxPointsPlugin();

      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

   private:
      ignition::math::Angle AngleMin() const;

      ignition::math::Angle AngleMax() const;

      double GetAngleResolution() const GAZEBO_DEPRECATED(7.0);

      double AngleResolution() const;

      double GetRangeMin() const GAZEBO_DEPRECATED(7.0);

      double RangeMin() const;

      double GetRangeMax() const GAZEBO_DEPRECATED(7.0);

      double RangeMax() const;

      double GetRangeResolution() const GAZEBO_DEPRECATED(7.0);

      double RangeResolution() const;

      int GetRayCount() const GAZEBO_DEPRECATED(7.0);

      int RayCount() const;

      int GetRangeCount() const GAZEBO_DEPRECATED(7.0);

      int RangeCount() const;

      int GetVerticalRayCount() const GAZEBO_DEPRECATED(7.0);

      int VerticalRayCount() const;

      int GetVerticalRangeCount() const GAZEBO_DEPRECATED(7.0);

      int VerticalRangeCount() const;

      ignition::math::Angle VerticalAngleMin() const;

      ignition::math::Angle VerticalAngleMax() const;

      double GetVerticalAngleResolution() const GAZEBO_DEPRECATED(7.0);

      double VerticalAngleResolution() const;

   protected:
      virtual void OnNewLaserScans();

   private:
      void InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                          boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape);

      void InitializeScan(msgs::LaserScan *&scan);

      void SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame, const std::string &child_frame);

      boost::shared_ptr<physics::LivoxOdeMultiRayShape> rayShape;
      gazebo::physics::CollisionPtr laserCollision;
      physics::EntityPtr parentEntity;
      transport::PublisherPtr scanPub;

      sdf::ElementPtr sdfPtr;
      transport::NodePtr node;
      msgs::LaserScanStamped laserMsg;
      gazebo::sensors::SensorPtr raySensor;
      std::vector<AviaRotateInfo> aviaInfos;

      gazebo_ros::Node::SharedPtr node_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_pub;
      rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub;
      
      std::string parent_name;
      std::string child_name;
      int64_t samplesStep = 0;
      int64_t currStartIndex = 0;
      int64_t maxPointSize = 1000;
      int64_t downSample = 1;
      int xfer_format = 1;  // 数据格式：0=PointCloud2, 1=Livox私有格式

      double maxDist = 40.0;
      double minDist = 0.1;
      
      // 缓存变量，避免频繁创建和销毁大型对象
      std::vector<std::pair<int, AviaRotateInfo>> points_pair_cache;
      livox_ros_driver2::msg::CustomMsg livox_msg;  // 预分配的Livox消息对象
      sensor_msgs::msg::PointCloud2 cloud2_msg;    // 预分配的PointCloud2消息对象
      
      // 运行时长统计相关变量
      struct PerformanceStats {
          uint64_t count = 0;
          double total_time = 0.0;
          double min_time = 0.0;
          double max_time = 0.0;
          double avg_time = 0.0;
          
          void update(double time) {
              count++;
              total_time += time;
              if (count == 1 || time < min_time) min_time = time;
              if (count == 1 || time > max_time) max_time = time;
              avg_time = total_time / count;
          }
          
          void reset() {
              count = 0;
              total_time = 0.0;
              min_time = 0.0;
              max_time = 0.0;
              avg_time = 0.0;
          }
      };
      
      // 主要阶段统计
      PerformanceStats on_new_laser_scans_stats;   // OnNewLaserScans函数总统计
      PerformanceStats initialize_rays_stats;      // InitializeRays函数耗时统计
      PerformanceStats ray_update_stats;           // rayShape->Update()耗时统计
      PerformanceStats message_init_stats;         // 消息初始化耗时统计
      PerformanceStats process_points_stats;       // 点云处理总耗时统计
      PerformanceStats message_publish_stats;      // 消息发布耗时统计
      
      // 点云处理细分阶段统计
      PerformanceStats points_calculation_stats;   // 点坐标计算耗时统计
      PerformanceStats message_assemble_stats;     // 消息组装耗时统计
      PerformanceStats thread_synchronize_stats;   // 线程同步耗时统计
      
      std::chrono::steady_clock::time_point last_stats_print;  // 上次输出统计结果的时间
   };

} // namespace gazebo

#endif // SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H


