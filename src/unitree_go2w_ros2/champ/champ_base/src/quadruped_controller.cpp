/*
Copyright (c) 2019-2020, Juan Miguel Jimeno

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <quadruped_controller.h>

champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time& time)
{
  return time.nanoseconds() / 1000ul;
}

QuadrupedController::QuadrupedController():
    Node("quadruped_controller_node",rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)),
    clock_(*this->get_clock()),
    body_controller_(base_),
    leg_controller_(base_, rosTimeToChampTime(clock_.now())),
    kinematics_(base_),
    enable_pose_compensation_(true),
    compensation_gain_(0.2),
    max_compensation_height_(0.1)
{
    std::string joint_control_topic = "joint_group_position_controller/command";
    std::string knee_orientation;
    std::string urdf = "";

    double loop_rate = 200.0;

    this->get_parameter("gait.pantograph_leg",         gait_config_.pantograph_leg);
    this->get_parameter("gait.max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    this->get_parameter("gait.max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    this->get_parameter("gait.com_x_translation",      gait_config_.com_x_translation);
    this->get_parameter("gait.swing_height",           gait_config_.swing_height);
    this->get_parameter("gait.stance_depth",           gait_config_.stance_depth);
    this->get_parameter("gait.stance_duration",        gait_config_.stance_duration);
    this->get_parameter("gait.nominal_height",         gait_config_.nominal_height);
    this->get_parameter("gait.knee_orientation",       knee_orientation);
    this->get_parameter("publish_foot_contacts",       publish_foot_contacts_);
    this->get_parameter("publish_joint_states",        publish_joint_states_);
    this->get_parameter("publish_joint_control",       publish_joint_control_);
    this->get_parameter("gazebo",                      in_gazebo_);
    this->get_parameter("joint_controller_topic",      joint_control_topic);
    this->get_parameter("loop_rate",                   loop_rate);
    this->get_parameter("urdf",                        urdf);
    
    // 姿态补偿参数
    this->get_parameter("enable_pose_compensation",    enable_pose_compensation_);
    this->get_parameter("compensation_gain",           compensation_gain_);
    this->get_parameter("max_compensation_height",     max_compensation_height_);
    
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel/smooth", 10, std::bind(&QuadrupedController::cmdVelCallback_, this,  std::placeholders::_1));
    cmd_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "body_pose", 1,  std::bind(&QuadrupedController::cmdPoseCallback_, this,  std::placeholders::_1));
    
    // IMU订阅用于姿态补偿
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, std::bind(&QuadrupedController::imuCallback_, this,  std::placeholders::_1));
    
    if(publish_joint_control_)
    {
        joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_control_topic, 10);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

    if(publish_foot_contacts_ && !in_gazebo_)
    {
        foot_contacts_publisher_   = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);
    }

    gait_config_.knee_orientation = knee_orientation.c_str();
    
    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromString(base_, this->get_node_parameters_interface(), urdf);
    joint_names_ = champ::URDF::getJointNames(this->get_node_parameters_interface());
    std::chrono::milliseconds period(static_cast<int>(1000/loop_rate));

    loop_timer_ = this->create_wall_timer(
         std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&QuadrupedController::controlLoop_, this));
    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedController::controlLoop_()
{
    float target_joint_positions[12];
    geometry::Transformation target_foot_positions[4];
    bool foot_contacts[4];

    body_controller_.poseCommand(target_foot_positions, req_pose_);

    leg_controller_.velocityCommand(target_foot_positions, req_vel_, rosTimeToChampTime(clock_.now()));
    
    // 应用姿态补偿
    applyPoseCompensation_(target_foot_positions);
    
    kinematics_.inverse(target_joint_positions, target_foot_positions);

    publishFootContacts_(foot_contacts);
    publishJoints_(target_joint_positions);
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void QuadrupedController::cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg)
{   
    
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    req_pose_.orientation.roll = roll;
    req_pose_.orientation.pitch = pitch;
    req_pose_.orientation.yaw = yaw;

    req_pose_.position.x = msg->position.x;
    req_pose_.position.y = msg->position.y;
    req_pose_.position.z = msg->position.z +  gait_config_.nominal_height;
}

void QuadrupedController::publishJoints_(float target_joints[12])
{   
    if(publish_joint_control_)
    {
        trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = clock_.now();
        joints_cmd_msg.header.stamp.sec = 0;
        joints_cmd_msg.header.stamp.nanosec = 0;
        
        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        point.time_from_start = rclcpp::Duration::from_seconds(1.0 / 60.0);
        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = target_joints[i];
        }

        joints_cmd_msg.points.push_back(point);
        joint_commands_publisher_->publish(joints_cmd_msg);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        sensor_msgs::msg::JointState joints_msg;

        joints_msg.header.stamp = clock_.now();

        joints_msg.name.resize(joint_names_.size());
        joints_msg.position.resize(joint_names_.size());
        joints_msg.name = joint_names_;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {    
            joints_msg.position[i]= target_joints[i];
        }

        joint_states_publisher_->publish(joints_msg);
    }
}

void QuadrupedController::publishFootContacts_(bool foot_contacts[4])
{
    if(publish_foot_contacts_ && !in_gazebo_)
    {
        champ_msgs::msg::ContactsStamped contacts_msg;
        contacts_msg.header.stamp = clock_.now();
        contacts_msg.contacts.resize(4);
        
        std::string s2;
       for(size_t i = 0; i < 4; i++)
        {
            //This is only published when there's no feedback on the robot
            //that a leg is in contact with the ground
            //For such cases, we use the stance phase in the gait for foot contacts
            contacts_msg.contacts[i] = base_.legs[i]->gait_phase();
            s2.append(std::to_string(contacts_msg.contacts[i]) + " ");
        }
        foot_contacts_publisher_->publish(contacts_msg);
    }
}

void QuadrupedController::imuCallback_(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    last_imu_ = msg;
}

void QuadrupedController::applyPoseCompensation_(geometry::Transformation (&foot_positions)[4])
{
    if (!enable_pose_compensation_ || last_imu_ == nullptr) {
        return;
    }

    // 从IMU数据提取姿态角
    tf2::Quaternion imu_quat(
        last_imu_->orientation.x,
        last_imu_->orientation.y,
        last_imu_->orientation.z,
        last_imu_->orientation.w);
    
    tf2::Matrix3x3 imu_rotation(imu_quat);
    double roll, pitch, yaw;
    imu_rotation.getRPY(roll, pitch, yaw);

    // 计算姿态补偿量
    double pitch_compensation = pitch * compensation_gain_;
    double roll_compensation = roll * compensation_gain_;

    // 限制最大补偿高度
    pitch_compensation = std::max(-max_compensation_height_ * 2, std::min(max_compensation_height_ * 2, pitch_compensation));
    roll_compensation = std::max(-max_compensation_height_, std::min(max_compensation_height_, roll_compensation));

    // 应用姿态补偿到四条腿
    // 前腿索引: 0 (左前), 1 (右前)
    // 后腿索引: 2 (左后), 3 (右后)
    
    // 俯仰补偿 (pitch): 身体前倾时前腿升高，后腿降低（恢复平衡）
    foot_positions[0].Translate(- pitch_compensation,   0, -pitch_compensation);  // 左前腿
    foot_positions[1].Translate(- pitch_compensation,   0, -pitch_compensation);  // 右前腿
    foot_positions[2].Translate(pitch_compensation, 0,  pitch_compensation);   // 左后腿
    foot_positions[3].Translate(pitch_compensation, 0,  pitch_compensation);   // 右后腿

    // 横滚补偿 (roll): 身体左倾时左腿升高，右腿降低（恢复平衡）
    foot_positions[0].Translate(0, roll_compensation / 2, roll_compensation);    // 左前腿
    foot_positions[2].Translate(0, roll_compensation / 2, roll_compensation);    // 左后腿
    foot_positions[1].Translate(0, - roll_compensation / 2, -roll_compensation);   // 右前腿
    foot_positions[3].Translate(0, - roll_compensation / 2, -roll_compensation);   // 右后腿

    RCLCPP_DEBUG(this->get_logger(), "姿态补偿: pitch=%.3f, roll=%.3f", pitch_compensation, roll_compensation);
}
