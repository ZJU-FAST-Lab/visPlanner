#include <bezier_predict/bezier_predict.h>
#include <bezier_predict/predictor.h>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
using namespace std;    
using namespace Eigen;


namespace ego_planner
{
// Predictor
  void Predictor::setEnvironment(const GridMap::Ptr &map)
  {
    this->grid_map_ = map;
  }

  void Predictor::setParam(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    bool simulation_flag;
    double fov;
    nh.param("bezier_predict/simulation", simulation_flag, true);
    nh.param("bezier_predict/simulation_fov", fov, -1.0);
    nh.param("bezier_predict/simulation_fov_max_dist", max_dis_, -1.0);
    nh.param("bezier_predict/simulation_fov_min_dist", min_dis_, -1.0);
    nh.param("bezier_predict/freq_donw", freq_donw_, -1);

    if(simulation_flag)
      target_odom_sub_ = nh.subscribe("simulator_target_odom", 1, &Predictor::SimulatorTargetOdometryCallback, this, ros::TransportHints().tcpNoDelay());
    else
      target_odom_sub_ = nh.subscribe("realworld_target_odom", 1, &Predictor::RealworldTargetOdometryCallback, this, ros::TransportHints().tcpNoDelay());

    odom_sub_ = nh.subscribe("odom_world", 1, &Predictor::odometryCallback, this, ros::TransportHints().tcpNoDelay());
    predict_list_pub_ = nh.advertise<visualization_msgs::Marker>("predict_points_list", 2);
    broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);

    visualization_ = vis;

    have_odom_ = false;
    list_max_size_ = 30;
    count_ = 0;

    double yaw_threshold_degree = fov / 2.0;
    yaw_threshold_ = yaw_threshold_degree / 180 * M_PI;
    // max_dis_ = 7.0;
    // min_dis_ = 0.5;
  }

  void Predictor::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_now_(0) = msg->pose.pose.position.x;
    odom_now_(1) = msg->pose.pose.position.y;
    odom_now_(2) = msg->pose.pose.position.z;

    Eigen::Quaterniond odom_orient;
    odom_orient.w() = msg->pose.pose.orientation.w;
    odom_orient.x() = msg->pose.pose.orientation.x;
    odom_orient.y() = msg->pose.pose.orientation.y;
    odom_orient.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d rot_x = odom_orient.toRotationMatrix().block(0, 0, 3, 1);
    yaw_now_ = atan2(rot_x(1), rot_x(0));

    have_odom_ = true;
  }

  void Predictor::RealworldTargetOdometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if( !have_odom_ )
      return;

    Eigen::Vector4d odom_pos;
    odom_pos(0) = msg->pose.pose.position.x;
    odom_pos(1) = msg->pose.pose.position.y;
    odom_pos(2) = msg->pose.pose.position.z;
    odom_pos(3) = ros::Time::now().toSec();

    Eigen::Vector3d drone_pos = odom_now_;
    Eigen::Vector3d swarm_prid = odom_pos.block(0,0,3,1);
    Eigen::Vector3d dist_vec = swarm_prid - drone_pos;

    target_list_.push_back(odom_pos);

    if( target_list_.size() > list_max_size_ )
      target_list_.erase(target_list_.begin());

    updateAndVisualPredict();
  }


  void Predictor::SimulatorTargetOdometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if( !have_odom_ )
      return;

    if( target_list_.size() < list_max_size_ )
    {
      Eigen::Vector4d odom_pos;
      odom_pos(0) = msg->pose.pose.position.x;
      odom_pos(1) = msg->pose.pose.position.y;
      odom_pos(2) = msg->pose.pose.position.z;
      odom_pos(3) = ros::Time::now().toSec();
      target_list_.push_back(odom_pos);
      return;
    }
  
    double resolution_inv = grid_map_->getResolution_inv();

    count_++;
    if( count_ == freq_donw_ )
    {
      count_ = 0;
      Eigen::Vector4d odom_pos;
      odom_pos(0) = msg->pose.pose.position.x;
      odom_pos(1) = msg->pose.pose.position.y;
      odom_pos(2) = msg->pose.pose.position.z;
      odom_pos(3) = ros::Time::now().toSec();

      Eigen::Vector3d drone_pos = odom_now_;
      Eigen::Vector3d swarm_prid = odom_pos.block(0,0,3,1);
      Eigen::Vector3d dist_vec = swarm_prid - drone_pos;


      // 增加预判，如果要看的飞机所处的位置距离我轨迹方向角度太大就不去了，难度太大
      double target_yaw = atan2(dist_vec(1), dist_vec(0));
      double now_yaw = yaw_now_;
      double d_yaw = now_yaw - target_yaw;

      if( d_yaw > M_PI )
        d_yaw -= ( 2 * M_PI );
      else if( d_yaw < -M_PI )
        d_yaw += ( 2 * M_PI );
      else{}

      if( abs(d_yaw) > yaw_threshold_ )
        return;

      double dist = sqrt(dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1));
      // 距离判断
      if( dist < max_dis_ && dist > min_dis_ )
      {
        // 3. collision check
        int check_step = dist * resolution_inv * 2;
        Eigen::Vector3d step = dist_vec / check_step;
        Eigen::Vector3d pos_temp = drone_pos;
        bool check_success = true;

        for( int j = 1; j < check_step; j++ )
        {
          pos_temp += step;
          if( grid_map_->isKnownOccupied(pos_temp) ) // occ
          {
            check_success = false;
            break;
          }
        }

        if( check_success )
        {
          // std::cout << "tracking!!!" << std::endl;

          target_list_.push_back(odom_pos);

          if( target_list_.size() > list_max_size_ )
            target_list_.erase(target_list_.begin());

          updateAndVisualPredict();
        }

      }
    }
  }

  bool Predictor::generatePredict( std::vector<Eigen::Vector3d> &sample_list, std::vector<double> &sample_time_list_ros )
  {
    if( target_list_.size() < 15 )
      return false;

    int bezier_flag = bezierpredict_.TrackingGeneration(5,5,target_list_);
    if( bezier_flag == 0 )
    {
      // dt 0.05s, duration 2.5s
      std::vector<Eigen::Vector3d> sample_pos;
      sample_pos = bezierpredict_.SamplePoslist_bezier(_PREDICT_SEG);
      int sample_size = sample_pos.size();
      // double time = target_list_[sample_size-1](3);

      for( size_t i = 0; i < sample_pos.size(); i++ )
      {
        sample_list.push_back( sample_pos[i] );
        // sample_time_list_ros.push_back( time + i * 0.05 );
      }
      return true;
    }
    else
    {
      return false;
    }
  }

  void Predictor::updateAndVisualPredict()
  {
    sample_list_.clear();
    sample_time_list_ros_.clear();
    if( generatePredict( sample_list_, sample_time_list_ros_ ) )
    {
      Eigen::MatrixXd ctrl_pts;
      double ts = 0.1;
      std::vector<Eigen::Vector3d> start_end_derivatives;
      start_end_derivatives.push_back(Eigen::Vector3d::Zero());
      start_end_derivatives.push_back(Eigen::Vector3d::Zero());
      start_end_derivatives.push_back(Eigen::Vector3d::Zero());
      start_end_derivatives.push_back(Eigen::Vector3d::Zero());
      UniformBspline::parameterizeToBspline(ts, sample_list_, start_end_derivatives, ctrl_pts);

      UniformBspline pos_traj(ctrl_pts, 3, ts);
      double duration = sample_list_.size() * ts;
      std::vector<Eigen::Vector3d> pos_list;

      // pub
      traj_utils::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = ros::Time::now();
      bspline.drone_id = 1;
      // bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = pos_traj.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = pos_traj.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      broadcast_bspline_pub_.publish(bspline);

      Eigen::Vector4d color(0, 1, 0.5, 1);
      visualization_->displaySphereList(predict_list_pub_, sample_list_, 0.8, color, 10);
    }
  }
}