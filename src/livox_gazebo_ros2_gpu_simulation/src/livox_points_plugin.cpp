#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/logging.hpp>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>// Store the latest laser scans into laserMsg
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <chrono>
#include <iomanip> // 用于std::fixed和std::setprecision
#include "ros2_livox/livox_points_plugin.h"
#include "ros2_livox/csv_reader.hpp"
#include "ros2_livox/livox_ode_multiray_shape.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <thread>
#include <future>
#include <vector>
#include <algorithm>

namespace gazebo
{

    GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

    LivoxPointsPlugin::LivoxPointsPlugin() {}

    LivoxPointsPlugin::~LivoxPointsPlugin() {}

    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos)
    {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;
        for (auto &data : datas)
        {
            if (data.size() == 3)
            {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * deg_2_rad;
                avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2; //转化成标准的右手系角度
            } else {
            RCLCPP_ERROR(rclcpp::get_logger("convertDataToRotateInfo"), "data size is not 3!");
        }
        }
    }

    void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf)
    {
        node_ = gazebo_ros::Node::Get(sdf);
        
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "load csv file name: %s", file_name.c_str());
        if (!CsvReader::ReadCsvFile(file_name, datas))
        {   
            RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "cannot get csv file! %s will return !", file_name.c_str());
            return;
        }
        sdfPtr = sdf;
        auto rayElem = sdfPtr->GetElement("ray");
        auto scanElem = rayElem->GetElement("scan");
        auto rangeElem = rayElem->GetElement("range");


        raySensor = _parent;
        auto sensor_pose = raySensor->Pose();
        auto curr_scan_topic = sdf->Get<std::string>("topic");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "ros topic name: %s", curr_scan_topic.c_str());

        // 读取xfer_format参数，默认为0（PointCloud2格式）
        xfer_format = 1;
        if (sdf->HasElement("xfer_format")) {
            xfer_format = sdf->Get<int>("xfer_format");
        }
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "xfer_format: %d", xfer_format);

        child_name = raySensor->Name();
        parent_name = raySensor->ParentName();
        size_t delimiter_pos = parent_name.find("::");
        parent_name = parent_name.substr(delimiter_pos + 2);

        node = transport::NodePtr(new transport::Node());
        node->Init(raySensor->WorldName());
        
        // 根据xfer_format参数决定发布哪种格式
        if (xfer_format == 0) {
            // PointCloud2格式
            cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic, 10);
            RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "Publishing PointCloud2 format on topic: %s", curr_scan_topic.c_str());
        } else if (xfer_format == 1) {
            // Livox私有格式
            custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, 10);
            RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "Publishing Livox CustomMsg format on topic: %s", curr_scan_topic.c_str());
        } else {
            // 默认使用PointCloud2格式
            cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic, 10);
            RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "Invalid xfer_format, defaulting to PointCloud2 format on topic: %s", curr_scan_topic.c_str());
        }

        scanPub = node->Advertise<msgs::LaserScanStamped>(curr_scan_topic+"laserscan", 50);

        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "scan info size: %ld", aviaInfos.size());
        maxPointSize = aviaInfos.size();

        RayPlugin::Load(_parent, sdfPtr);
        laserMsg.mutable_scan()->set_frame(_parent->ParentName());
        // parentEntity = world->GetEntity(_parent->ParentName());
        parentEntity = this->world->EntityByName(_parent->ParentName());
        //SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());
        auto physics = world->Physics();
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());
        rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);
        samplesStep = sdfPtr->Get<int>("samples");
        downSample = sdfPtr->Get<int>("downsample");
        if (downSample < 1)
        {
            downSample = 1;
        }
        
        // 初始化运行时长统计的时间戳
        last_stats_print = std::chrono::steady_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "sample: %ld", samplesStep);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "downsample: %ld", downSample);
        rayShape->RayShapes().reserve(samplesStep / downSample);
        rayShape->Load(sdfPtr);
        rayShape->Init();
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");
        
        // 预分配缓存变量的内存
        points_pair_cache.reserve(samplesStep / downSample);
        
        if (xfer_format == 0) {
            // PointCloud2格式 - 预初始化消息字段
            cloud2_msg.header.frame_id = raySensor->Name();
            cloud2_msg.height = 1;
            cloud2_msg.is_dense = false;
            cloud2_msg.point_step = 16;
            
            // 定义字段（x, y, z, intensity）
            sensor_msgs::msg::PointField field_x, field_y, field_z, field_intensity;
            field_x.name = "x";
            field_x.offset = 0;
            field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field_x.count = 1;

            field_y.name = "y";
            field_y.offset = 4;
            field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field_y.count = 1;

            field_z.name = "z";
            field_z.offset = 8;
            field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field_z.count = 1;

            field_intensity.name = "intensity";
            field_intensity.offset = 12;
            field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field_intensity.count = 1;

            cloud2_msg.fields = {field_x, field_y, field_z, field_intensity};
            cloud2_msg.data.reserve(1024 * 1024); // 预分配1MB数据空间
        } else if (xfer_format == 1) {
            // Livox私有格式 - 预初始化消息
            livox_msg.header.frame_id = raySensor->Name();
            livox_msg.points.reserve(10000); // 预分配足够的点云空间
        }
        
        // 添加射线
        auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start_point, end_point;
        for (int j = 0; j < samplesStep; j += downSample)
        {
            int index = j % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            rayShape->AddRay(start_point, end_point);
        }
    }
void LivoxPointsPlugin::OnNewLaserScans()
{
    // 开始统计OnNewLaserScans函数总运行时间
    auto on_new_laser_scans_start = std::chrono::steady_clock::now();
    
    if (rayShape)
    {
        // 使用缓存的points_pair避免重复分配内存
        points_pair_cache.clear();
        
        // 计时InitializeRays函数
        auto initialize_rays_start = std::chrono::steady_clock::now();
        InitializeRays(points_pair_cache, rayShape);
        auto initialize_rays_end = std::chrono::steady_clock::now();
        double initialize_rays_time = std::chrono::duration<double, std::milli>(initialize_rays_end - initialize_rays_start).count();
        initialize_rays_stats.update(initialize_rays_time);
        
        // 计时rayShape->Update()
        auto ray_update_start = std::chrono::steady_clock::now();
        rayShape->Update();
        auto ray_update_end = std::chrono::steady_clock::now();
        double ray_update_time = std::chrono::duration<double, std::milli>(ray_update_end - ray_update_start).count();
        ray_update_stats.update(ray_update_time);

        // 计时激光扫描消息初始化
        auto message_init_start = std::chrono::steady_clock::now();
        msgs::Set(laserMsg.mutable_time(), world->SimTime());
        msgs::LaserScan *scan = laserMsg.mutable_scan();
        InitializeScan(scan);
        auto message_init_end = std::chrono::steady_clock::now();
        double message_init_time = std::chrono::duration<double, std::milli>(message_init_end - message_init_start).count();
        message_init_stats.update(message_init_time);

        int count = 0;
        
        // 开始统计点云处理时间
        auto process_points_start = std::chrono::steady_clock::now();
        
        boost::chrono::high_resolution_clock::time_point start_time = boost::chrono::high_resolution_clock::now();

        // 根据xfer_format参数设置消息头
        if (xfer_format == 0) {
            cloud2_msg.header.stamp = node_->get_clock()->now();
            cloud2_msg.data.clear(); // 清空数据，保留预分配的内存
        } else if (xfer_format == 1) {
            livox_msg.header.stamp = node_->get_clock()->now();
            livox_msg.points.clear(); // 清空数据，保留预分配的内存
        }

        std::vector<uint8_t> cloud_data;
        if (xfer_format == 0) {
            cloud_data.reserve(points_pair_cache.size() * 16);
        }

        // 定义点云处理辅助函数
        auto processPoints = [&](const std::vector<std::pair<int, AviaRotateInfo>>& sub_points, 
                                int thread_id) -> std::vector<uint8_t> {
            std::vector<uint8_t> thread_cloud_data;
            if (xfer_format == 0) {
                thread_cloud_data.reserve(sub_points.size() * 16); // 预分配内存
            }
            
            for (const auto &pair : sub_points) {
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);

                if (range >= RangeMax() || range <= RangeMin())
                    range = 0;

                // 计算点坐标
                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;

                // 根据xfer_format参数填充相应的数据格式
                if (xfer_format == 0) {
                    // 填充 PointCloud2 数据
                    float x = point.X();
                    float y = point.Y();
                    float z = point.Z();
                    float inten = intensity;

                    const uint8_t *x_ptr = reinterpret_cast<const uint8_t*>(&x);
                    const uint8_t *y_ptr = reinterpret_cast<const uint8_t*>(&y);
                    const uint8_t *z_ptr = reinterpret_cast<const uint8_t*>(&z);
                    const uint8_t *inten_ptr = reinterpret_cast<const uint8_t*>(&inten);
                    
                    thread_cloud_data.insert(thread_cloud_data.end(), x_ptr, x_ptr + 4);
                    thread_cloud_data.insert(thread_cloud_data.end(), y_ptr, y_ptr + 4);
                    thread_cloud_data.insert(thread_cloud_data.end(), z_ptr, z_ptr + 4);
                    thread_cloud_data.insert(thread_cloud_data.end(), inten_ptr, inten_ptr + 4);
                }
            }
            return thread_cloud_data;
        };
        
        // 定义CustomMsg处理辅助函数
        auto processCustomPoints = [&](const std::vector<std::pair<int, AviaRotateInfo>>& sub_points, 
                                     boost::chrono::high_resolution_clock::time_point thread_start_time) -> std::vector<livox_ros_driver2::msg::CustomPoint> {
            std::vector<livox_ros_driver2::msg::CustomPoint> thread_points;
            thread_points.reserve(sub_points.size());
            
            for (const auto &pair : sub_points) {
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);

                if (range >= RangeMax() || range <= RangeMin())
                    range = 0;

                // 计算点坐标
                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;

                // 填充 CustomMsg
                livox_ros_driver2::msg::CustomPoint p;
                p.x = point.X();
                p.y = point.Y();
                p.z = point.Z();
                p.reflectivity = intensity;

                // 计算时间偏移
                boost::chrono::high_resolution_clock::time_point end_time = boost::chrono::high_resolution_clock::now();
                boost::chrono::nanoseconds elapsed_time = boost::chrono::duration_cast<boost::chrono::nanoseconds>(end_time - thread_start_time);
                p.offset_time = elapsed_time.count();
                
                thread_points.push_back(p);
            }
            return thread_points;
        };
        
        // 使用多线程并行处理点云数据
        unsigned int num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) num_threads = 8; // 默认使用2线程，减少线程竞争
        
        // 避免线程数量超过点云数量，同时限制最大线程数
        num_threads = std::min(num_threads, std::min((unsigned int)points_pair_cache.size() / 1000, (unsigned int)4));
        if (num_threads < 1) num_threads = 1;
        
        if (xfer_format == 0) {
            // PointCloud2格式处理
            // 预分配足够的内存空间，避免多次重新分配
            cloud_data.reserve(points_pair_cache.size() * 16);
            
            // 使用并行算法处理点云数据
            // 注意：这里不创建子向量，直接传递迭代器范围以避免数据拷贝
            std::vector<std::future<void>> futures;
            futures.reserve(num_threads);
            
            // 划分点云数据到多个线程
            size_t chunk_size = points_pair_cache.size() / num_threads;
            size_t remainder = points_pair_cache.size() % num_threads;
            
            size_t start = 0;
            for (unsigned int i = 0; i < num_threads; i++) {
                size_t end = start + chunk_size + (i < remainder ? 1 : 0);
                if (start >= end) break;
                
                // 直接处理原向量的子范围，避免数据拷贝
                futures.push_back(std::async(std::launch::async, [this, start, end, &cloud_data]() {
                    // 计算这个线程负责的云数据起始位置
                    size_t data_start = start * 16;
                    
                    for (size_t j = start; j < end; j++) {
                        const auto &pair = this->points_pair_cache[j];
                        auto range = this->rayShape->GetRange(pair.first);
                        auto intensity = this->rayShape->GetRetro(pair.first);

                        if (range >= this->RangeMax() || range <= this->RangeMin())
                            range = 0;

                        // 计算点坐标
                        auto rotate_info = pair.second;
                        ignition::math::Quaterniond ray;
                        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                        auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                        auto point = range * axis;

                        // 直接将数据写入预分配的向量
                        float x = point.X();
                        float y = point.Y();
                        float z = point.Z();
                        float inten = intensity;
                        
                        // 计算当前点在云数据中的位置
                        size_t data_pos = data_start + (j - start) * 16;
                        
                        // 直接写入数据，避免使用insert
                        std::memcpy(&cloud_data[data_pos], &x, 4);
                        std::memcpy(&cloud_data[data_pos + 4], &y, 4);
                        std::memcpy(&cloud_data[data_pos + 8], &z, 4);
                        std::memcpy(&cloud_data[data_pos + 12], &inten, 4);
                    }
                }));
                
                start = end;
            }
            
            // 等待所有线程完成
            for (auto &future : futures) {
                future.get();
            }
        } else if (xfer_format == 1) {
            // CustomMsg格式处理
            // 预分配足够的内存空间
            livox_msg.points.reserve(points_pair_cache.size());
            
            // 使用并行算法处理点云数据
            std::vector<std::future<std::vector<livox_ros_driver2::msg::CustomPoint>>> futures;
            futures.reserve(num_threads);
            
            // 划分点云数据到多个线程
            size_t chunk_size = points_pair_cache.size() / num_threads;
            size_t remainder = points_pair_cache.size() % num_threads;
            
            size_t start = 0;
            for (unsigned int i = 0; i < num_threads; i++) {
                size_t end = start + chunk_size + (i < remainder ? 1 : 0);
                if (start >= end) break;
                
                // 使用共享的起始时间点
                auto thread_start_time = start_time;
                
                // 创建一个局部向量来存储线程结果
                futures.push_back(std::async(std::launch::async, [this, start, end, thread_start_time]() -> std::vector<livox_ros_driver2::msg::CustomPoint> {
                    std::vector<livox_ros_driver2::msg::CustomPoint> thread_points;
                    thread_points.reserve(end - start);
                    
                    for (size_t j = start; j < end; j++) {
                        const auto &pair = this->points_pair_cache[j];
                        auto range = this->rayShape->GetRange(pair.first);
                        auto intensity = this->rayShape->GetRetro(pair.first);

                        if (range >= this->RangeMax() || range <= this->RangeMin())
                            range = 0;

                        // 计算点坐标
                        auto rotate_info = pair.second;
                        ignition::math::Quaterniond ray;
                        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                        auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                        auto point = range * axis;

                        // 填充 CustomMsg
                        livox_ros_driver2::msg::CustomPoint p;
                        p.x = point.X();
                        p.y = point.Y();
                        p.z = point.Z();
                        p.reflectivity = intensity;
                        
                        // 计算时间偏移
                        boost::chrono::high_resolution_clock::time_point end_time = boost::chrono::high_resolution_clock::now();
                        boost::chrono::nanoseconds elapsed_time = boost::chrono::duration_cast<boost::chrono::nanoseconds>(end_time - thread_start_time);
                        p.offset_time = elapsed_time.count();
                        
                        thread_points.push_back(p);
                    }
                    return thread_points;
                }));
                
                start = end;
            }
            
            // 收集结果
            for (auto &future : futures) {
                auto thread_points = future.get();
                livox_msg.points.insert(livox_msg.points.end(), thread_points.begin(), thread_points.end());
            }
            count = livox_msg.points.size();
        } else {
            // 单线程处理其他格式
            for (auto &pair : points_pair_cache)
            {
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);

                if (range >= RangeMax() || range <= RangeMin())
                    range = 0;

                // 计算点坐标
                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;

                // 根据xfer_format参数填充相应的数据格式
                if (xfer_format == 0) {
                    // 填充 PointCloud2 数据
                    float x = point.X();
                    float y = point.Y();
                    float z = point.Z();
                    float inten = intensity;

                    auto write_float = [&](float val) {
                        const uint8_t *ptr = reinterpret_cast<const uint8_t*>(&val);
                        cloud_data.insert(cloud_data.end(), ptr, ptr + 4);
                    };
                    write_float(x);
                    write_float(y);
                    write_float(z);
                    write_float(inten);
                    cloud2_msg.width++;
                }
                count++;
            }
        }

        // 计算点云处理时间并更新统计
        auto process_points_end = std::chrono::steady_clock::now();
        double process_points_time = std::chrono::duration<double, std::milli>(process_points_end - process_points_start).count();
        process_points_stats.update(process_points_time);
        
        // 统计各个细分阶段的耗时
        // 点坐标计算耗时统计（在点云处理过程中统计）
        auto points_calculation_start = std::chrono::steady_clock::now();
        // 这里可以添加具体的点坐标计算统计逻辑
        auto points_calculation_end = std::chrono::steady_clock::now();
        double points_calculation_time = std::chrono::duration<double, std::milli>(points_calculation_end - points_calculation_start).count();
        points_calculation_stats.update(points_calculation_time);
        
        // 消息组装耗时统计
        auto message_assemble_start = std::chrono::steady_clock::now();
        // 这里可以添加具体的消息组装统计逻辑
        auto message_assemble_end = std::chrono::steady_clock::now();
        double message_assemble_time = std::chrono::duration<double, std::milli>(message_assemble_end - message_assemble_start).count();
        message_assemble_stats.update(message_assemble_time);
        
        // 线程同步耗时统计
        auto thread_synchronize_start = std::chrono::steady_clock::now();
        // 这里可以添加具体的线程同步统计逻辑
        auto thread_synchronize_end = std::chrono::steady_clock::now();
        double thread_synchronize_time = std::chrono::duration<double, std::milli>(thread_synchronize_end - thread_synchronize_start).count();
        thread_synchronize_stats.update(thread_synchronize_time);
        
        // 记录消息发布开始时间
        auto message_publish_start = std::chrono::steady_clock::now();
        
        // 发布消息
        if (scanPub && scanPub->HasConnections()) 
            scanPub->Publish(laserMsg);
            
        // 发布点云数据
        if (xfer_format == 0) {
            // 设置 PointCloud2 消息的宽度和行步长
            cloud2_msg.width = cloud_data.size() / cloud2_msg.point_step;
            cloud2_msg.row_step = cloud2_msg.width * cloud2_msg.point_step;

            // 将数据复制到 PointCloud2 消息中
            cloud2_msg.data.swap(cloud_data);
            if (cloud2_pub) {
                cloud2_pub->publish(cloud2_msg);
            }
        } else if (xfer_format == 1) {
            // 发布 Livox 消息
            livox_msg.point_num = count;
            if (custom_pub) {
                custom_pub->publish(livox_msg);
            }
        }
        
        // 计算消息发布时间并更新统计
        auto message_publish_end = std::chrono::steady_clock::now();
        double message_publish_time = std::chrono::duration<double, std::milli>(message_publish_end - message_publish_start).count();
        message_publish_stats.update(message_publish_time);
        
        // 计算OnNewLaserScans函数总运行时间并更新统计
        auto on_new_laser_scans_end = std::chrono::steady_clock::now();
        double on_new_laser_scans_time = std::chrono::duration<double, std::milli>(on_new_laser_scans_end - on_new_laser_scans_start).count();
        on_new_laser_scans_stats.update(on_new_laser_scans_time);
        
        // 检查是否需要输出统计结果（每60秒一次）
        if (std::chrono::steady_clock::now() - last_stats_print >= std::chrono::seconds(60)) {
            // 输出统计结果
        std::cout << "=== 运行时长统计 (每60秒输出一次) ===\n";
        std::cout << "OnNewLaserScans函数总耗时: " << std::fixed << std::setprecision(3);
        std::cout << "次数: " << on_new_laser_scans_stats.count;
        std::cout << ", 平均: " << on_new_laser_scans_stats.avg_time << "ms";
        std::cout << ", 最小: " << on_new_laser_scans_stats.min_time << "ms";
        std::cout << ", 最大: " << on_new_laser_scans_stats.max_time << "ms\n";
        
        // 主要阶段细分
        std::cout << "  InitializeRays耗时: " << std::fixed << std::setprecision(3);
        std::cout << "次数: " << initialize_rays_stats.count;
        std::cout << ", 平均: " << initialize_rays_stats.avg_time << "ms";
        std::cout << ", 最小: " << initialize_rays_stats.min_time << "ms";
        std::cout << ", 最大: " << initialize_rays_stats.max_time << "ms\n";
        
        std::cout << "  rayShape->Update()耗时: " << std::fixed << std::setprecision(3);
        std::cout << "次数: " << ray_update_stats.count;
        std::cout << ", 平均: " << ray_update_stats.avg_time << "ms";
        std::cout << ", 最小: " << ray_update_stats.min_time << "ms";
        std::cout << ", 最大: " << ray_update_stats.max_time << "ms\n";
        
        std::cout << "  消息初始化耗时: " << std::fixed << std::setprecision(3);
        std::cout << "次数: " << message_init_stats.count;
        std::cout << ", 平均: " << message_init_stats.avg_time << "ms";
        std::cout << ", 最小: " << message_init_stats.min_time << "ms";
        std::cout << ", 最大: " << message_init_stats.max_time << "ms\n";
        
        std::cout << "  点云处理总耗时: " << std::fixed << std::setprecision(3);
        std::cout << "次数: " << process_points_stats.count;
        std::cout << ", 平均: " << process_points_stats.avg_time << "ms";
        std::cout << ", 最小: " << process_points_stats.min_time << "ms";
        std::cout << ", 最大: " << process_points_stats.max_time << "ms\n";
        
        // 点云处理细分阶段
        std::cout << "    点坐标计算耗时: " << std::fixed << std::setprecision(3);
        std::cout << "次数: " << points_calculation_stats.count;
        std::cout << ", 平均: " << points_calculation_stats.avg_time << "ms";
        std::cout << ", 最小: " << points_calculation_stats.min_time << "ms";
        std::cout << ", 最大: " << points_calculation_stats.max_time << "ms\n";
        
        std::cout << "    消息组装耗时: " << std::fixed << std::setprecision(3);
        std::cout << "次数: " << message_assemble_stats.count;
        std::cout << ", 平均: " << message_assemble_stats.avg_time << "ms";
        std::cout << ", 最小: " << message_assemble_stats.min_time << "ms";
        std::cout << ", 最大: " << message_assemble_stats.max_time << "ms\n";
        
        std::cout << "    线程同步耗时: " << std::fixed << std::setprecision(3);
        std::cout << "次数: " << thread_synchronize_stats.count;
        std::cout << ", 平均: " << thread_synchronize_stats.avg_time << "ms";
        std::cout << ", 最小: " << thread_synchronize_stats.min_time << "ms";
        std::cout << ", 最大: " << thread_synchronize_stats.max_time << "ms\n";
        
        std::cout << "  消息发布耗时: " << std::fixed << std::setprecision(3);
        std::cout << "次数: " << message_publish_stats.count;
        std::cout << ", 平均: " << message_publish_stats.avg_time << "ms";
        std::cout << ", 最小: " << message_publish_stats.min_time << "ms";
        std::cout << ", 最大: " << message_publish_stats.max_time << "ms\n";
            
            std::cout << "=====================================\n";
            
            // 重置统计信息
            on_new_laser_scans_stats.reset();
            initialize_rays_stats.reset();
            ray_update_stats.reset();
            message_init_stats.reset();
            process_points_stats.reset();
            message_publish_stats.reset();
            points_calculation_stats.reset();
            message_assemble_stats.reset();
            thread_synchronize_stats.reset();
            
            // 更新上次输出时间
            last_stats_print = std::chrono::steady_clock::now();
        }
    }
}

//     void LivoxPointsPlugin::OnNewLaserScans()
// {
//     // Check if rayShape has been initialized
//     if (rayShape)
//     {
//         std::vector<std::pair<int, AviaRotateInfo>> points_pair;
//         // Initialize ray scan point pairs
//         InitializeRays(points_pair, rayShape);
//         rayShape->Update();

//         // Create laser scan message and set the timestamp
//         msgs::Set(laserMsg.mutable_time(), world->SimTime());
//         msgs::LaserScan *scan = laserMsg.mutable_scan();
//         InitializeScan(scan);

//         // Create a custom message pp_livox for publishing Livox CustomMsg type messages
//         livox_ros_driver2::msg::CustomMsg pp_livox;
//         pp_livox.header.stamp = node_->get_clock()->now();
//         pp_livox.header.frame_id = raySensor->Name();
//         int count = 0;
//         boost::chrono::high_resolution_clock::time_point start_time = boost::chrono::high_resolution_clock::now();

//         // For publishing PointCloud2 type messages
//         sensor_msgs::msg::PointCloud cloud;
//         cloud.header.stamp = node_->get_clock()->now();
//         cloud.header.frame_id = raySensor->Name();
//         auto &clouds = cloud.points;

//         // Iterate over ray scan point pairs
//         for (auto &pair : points_pair)
//         {
//             auto range = rayShape->GetRange(pair.first);
//             auto intensity = rayShape->GetRetro(pair.first);

//             // Handle out-of-range data
//             if (range >= RangeMax())
//             {
//                 range = 0;
//             }
//             else if (range <= RangeMin())
//             {
//                 range = 0;
//             }

//             // Calculate point cloud data
//             auto rotate_info = pair.second;
//             ignition::math::Quaterniond ray;
//             ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
//             auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
//             auto point = range * axis;

//             // Fill the CustomMsg point cloud message
//             livox_ros_driver2::msg::CustomPoint p;
//             p.x = point.X();
//             p.y = point.Y();
//             p.z = point.Z();
//             p.reflectivity = intensity;

//             // Fill the PointCloud point cloud message
//             clouds.emplace_back();
//             clouds.back().x = point.X();
//             clouds.back().y = point.Y();
//             clouds.back().z = point.Z();

//             // Fill the PointCloud point cloud message
//             clouds.emplace_back();
//             clouds.back().x = point.X();
//             clouds.back().y = point.Y();
//             clouds.back().z = point.Z();

//             // Calculate timestamp offset
//             boost::chrono::high_resolution_clock::time_point end_time = boost::chrono::high_resolution_clock::now();
//             boost::chrono::nanoseconds elapsed_time = boost::chrono::duration_cast<boost::chrono::nanoseconds>(end_time - start_time);
//             p.offset_time = elapsed_time.count();

//             // Add point cloud data to the CustomMsg message
//             pp_livox.points.push_back(p);
//             count++;
//         }

//         if (scanPub && scanPub->HasConnections()) scanPub->Publish(laserMsg);

//         // Set the number of point cloud data and publish the CustomMsg message
//         pp_livox.point_num = count;
//         custom_pub->publish(pp_livox);

//         // Publish PointCloud2 type message
//         sensor_msgs::msg::PointCloud2 cloud2;
//         sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
//         cloud2.header = cloud.header;
//         cloud2_pub->publish(cloud2);
//     }
// }

    void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                           boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape)
    {
        auto &rays = ray_shape->RayShapes();
        auto ray_size = rays.size();
        
        // 预先计算固定值，避免重复计算
        auto offset = laserCollision->RelativePose();
        auto offset_rot = offset.Rot();
        
        // 使用预分配的内存，避免频繁分配
        points_pair.clear();
        points_pair.reserve(ray_size);
        
        int64_t end_index = currStartIndex + samplesStep;
        long unsigned int ray_index = 0;
        
        // 批量处理射线，减少循环开销
        for (int k = currStartIndex; k < end_index && ray_index < ray_size; k += downSample)
        {
            auto index = k % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            
            // 预计算射线方向
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset_rot * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            
            // 计算起点和终点
            auto start_point = minDist * axis + offset.Pos();
            auto end_point = maxDist * axis + offset.Pos();
            
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, rotate_info);
            ray_index++;
        }
        
        currStartIndex += samplesStep;
    }

    void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan)
    {
        // Store the latest laser scans into laserMsg
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        scan->set_angle_min(AngleMin().Radian());
        scan->set_angle_max(AngleMax().Radian());
        scan->set_angle_step(AngleResolution());
        scan->set_count(RangeCount());

        scan->set_vertical_angle_min(VerticalAngleMin().Radian());
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());
        scan->set_vertical_angle_step(VerticalAngleResolution());
        scan->set_vertical_count(VerticalRangeCount());

        scan->set_range_min(RangeMin());
        scan->set_range_max(RangeMax());

        scan->clear_ranges();
        scan->clear_intensities();

        unsigned int rangeCount = RangeCount();
        unsigned int verticalRangeCount = VerticalRangeCount();

        for (unsigned int j = 0; j < verticalRangeCount; ++j)
        {
            for (unsigned int i = 0; i < rangeCount; ++i)
            {
                scan->add_ranges(0);
                scan->add_intensities(0);
            }
        }
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMin() const
    {
        if (rayShape)
            return rayShape->MinAngle();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->MaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

    double LivoxPointsPlugin::RangeMin() const
    {
        if (rayShape)
            return rayShape->GetMinRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

    double LivoxPointsPlugin::RangeMax() const
    {
        if (rayShape)
            return rayShape->GetMaxRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

    double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

    double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

    double LivoxPointsPlugin::RangeResolution() const
    {
        if (rayShape)
            return rayShape->GetResRange();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

    int LivoxPointsPlugin::RayCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

    int LivoxPointsPlugin::RangeCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount() * rayShape->GetScanResolution();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

    int LivoxPointsPlugin::VerticalRayCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

    int LivoxPointsPlugin::VerticalRangeCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
        }
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

    double LivoxPointsPlugin::VerticalAngleResolution() const
    {
        return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
    }


}
