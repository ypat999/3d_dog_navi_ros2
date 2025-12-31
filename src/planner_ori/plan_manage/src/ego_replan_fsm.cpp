#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    nh.param<std::string>("fsm/odometry_topic", odom_topic, "/Odometry");

    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.2), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.1), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe(odom_topic, 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);


    switch (target_type_)
    {
    //手动给予坐标点摸索
    case TARGET_TYPE::MANUAL_TARGET:
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
      break;
    case TARGET_TYPE::PRESET_TARGET:
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      cout << "planGlobalTrajbyGivenWps  in init" << endl;
      planGlobalTrajbyGivenWps();
      break;
    case TARGET_TYPE::REFENCE_PATH:
      //读取path话题生成全局轨迹
      path_sub_= nh.subscribe("/pct_path",1,&EGOReplanFSM::pathCallback,this);
      break;
    default:
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
      break;
    }
     
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      // visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }
  

  void EGOReplanFSM::pathCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses.empty())
    {
      ROS_WARN("[EGO] Received empty path from move_base.");
      return;
    }

    if (!have_odom_)
    {
      ROS_WARN("[EGO] No odometry yet, cannot plan global trajectory.");
      return;
    }
    // === 1. 提取并转换 waypoints ===
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    double min_dist = 0.8;  // 下采样：每隔 0.8m 保留一个点（避免点太密）
    Eigen::Vector3d last_wp;
    //添加最后终点
    end_pt_ << msg->poses.back().pose.position.x, msg->poses.back().pose.position.y, msg->poses.back().pose.position.z;
    bool first = true;

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z;  // A1 是地面机器人，强制 z=0（可改成 pose_stamped.pose.position.z）

      //第一个点，给入的是odom坐标
      if (first)
      {
        waypoints.push_back(wp);
        last_wp = wp;
        first = false;
        continue;
      }

      // 下采样：距离上一个点 > min_dist 才加入
      if ((wp - last_wp).norm() >= min_dist)
      {
        waypoints.push_back(wp);
        last_wp = wp;
      }
    }

    for (size_t i = 0; i < (size_t)msg->poses.size(); i++)
    {
      // visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }
    // === 2. 计划轨迹 ===
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, odom_pos_(2); //twilight: goal height
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    std::cout<<"run_flag"<<success<<std::endl;

    // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ){

        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
              

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
  
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();

    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }



  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // odom_topic
    odom_pos_= Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);//将机器人通过高度设置为，其高度的80*
    odom_vel_=Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, 0);
    // odom_acc_ = estimateAcc( msg );
    have_odom_ = true;
   //将当前位姿从odom坐标系转换到map坐标，如果存在tf变换关系的话
    try
    {
      // 则当前odom_pos_为base与map的关系
      tf::StampedTransform transform_odom2map;
      transform_odom2map.setIdentity();//将变换矩阵初始化为单位阵

      //将当前位姿从odom坐标系转换到map坐标，如果存在tf变换关系的话
      if (tf_listener_.waitForTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(3.0)));
      tf_listener_.lookupTransform("map", msg->header.frame_id,
                                  ros::Time(0), transform_odom2map);
      // 位置变换    
      tf::Point pt_odom(odom_pos_(0), odom_pos_(1), odom_pos_(2));
      tf::Point pt_map = transform_odom2map * pt_odom;
      odom_pos_= Eigen::Vector3d(pt_map.x(), pt_map.y(), pt_map.z());
                                
      // 速度变换（线速度需要旋转到 map 坐标系）
      tf::Vector3 vel_odom(odom_vel_(0), odom_vel_(1), odom_vel_(2));
      tf::Vector3 vel_map = transform_odom2map.getBasis() * vel_odom;  // 只旋转，不平移
      odom_vel_= Eigen::Vector3d(vel_map.x(), vel_map.y(), vel_map.z());

    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      return;
    }
    
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    /*定时器回调，定时查看当前的exec状态，以判断是否规划或是停止*/
    // cout << "execFSMCallback" << endl;
    MAX_EMERGENCY_STOP=5;
    static int emergency_stop_times=0;
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 10)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }
    switch (exec_state_)
    {
    case INIT:
    {
      // cout << "change state to INIT" <<endl;

      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      cout << " case GEN_NEW_TRAJ:" <<endl;
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);

      if (success)
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      cout << "!!!REPLAN_TRAJ" <<endl;
      if (planFromCurrentTraj())
      {
        // planFromCurrentTraj()
        // cout << "change state to EXEC_TRAJ in '!!!REPLAN_TRAJ'" <<endl;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // cout << "change state to REPLAN_TRAJ in '!!!REPLAN_TRAJ" <<endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      cout << "Executing... "<< endl;
      Eigen::Vector3d pos = odom_pos_;

      if ((end_pt_ - pos).norm() < 0.2)
      {
        cout << " near end position" << endl;
        cout << "### change state to WAIT_TARGET in '!!!EXEC_TRAJ'" << endl;
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else
      {
        cout << "### change state to REPLAN_TRAJ in '!!!EXEC_TRAJ'" << endl;

        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
        emergency_stop_times++;//累积进入停止时间，直到强制停止次大于100次，则恢复
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      ROS_ERROR("emergency_stop_times :%d", emergency_stop_times);

      if (emergency_stop_times >MAX_EMERGENCY_STOP){
        flag_escape_emergency_ = false;
        emergency_stop_times=0;
      }

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    info->start_pos_=odom_pos_;

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    cout<<" t_cur "<<t_cur<<endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);


    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

  // 到目标点才停止replan
    cout << " final plan success="<< endl;
    return true;
;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;

    info->start_pos_ = odom_pos_;

    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_total = info->duration_ ;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_total && t >= t_total) 
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    cout<<"current pos:"<<start_pt_.transpose()<<endl;
    cout<<"future target:"<<local_target_pt_.transpose()<<endl;

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false; //FIXME

    cout<<"plan_success: "<<plan_success;

    if (plan_success)
    {
      auto info = &planner_manager_->local_data_;
      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();//get_control_points();会向前得到t_cur时间段的轨迹
      bspline.pos_pts.reserve(pos_pts.cols());
      
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);
      // visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }
    //Todo一个更合理的导航失败切换
    else//如果规划失败马上停下，并开始
    {
      /* code */
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      flag_escape_emergency_ = true;
    }
    

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;
    info->start_pos_ = odom_pos_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

/**
 * @brief 获取局部目标点的函数
 * 该函数用于在全局路径上选择一个合适的局部目标点，用于局部路径规划
 */
  void EGOReplanFSM::getLocalTarget()
  {
    double t;  // 时间变量，用于遍历全局路径

    // 计算时间步长，基于规划范围、最大速度等因素
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;  // 初始化最小距离和对应时间
    // 遍历全局路径，寻找合适的局部目标点
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
        // 获取当前时间点的位置
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
        // 计算当前位置与起始点的距离
      double dist = (pos_t - start_pt_).norm();

        // 检查last_progress_time_是否有效
      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo: 处理last_progress_time_错误的情况
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

} // namespace ego_planner
