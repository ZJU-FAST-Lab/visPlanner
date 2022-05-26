#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include<fstream>

ros::Publisher pos_cmd_pub, fov_pub_;
ros::Publisher line_pub_, fasttracker_line_pub_, fasttracker_path_pub_;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
bool have_yaw_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

// fov visualize
double max_dis_ = 4.0;
double x_max_dis_gain_ = 0.64;
double y_max_dis_gain_ = 0.82;
visualization_msgs::Marker markerNode_fov;
visualization_msgs::Marker markerEdge_fov;
visualization_msgs::Marker marker_line, fast_marker_line;
std::vector<Eigen::Vector3d> fov_node;

int drone_id_;
Eigen::Vector3d target_odom_pos_{0, 0, 0};
bool target_odom_ = false;
nav_msgs::Path fasttracker_path_;

void fov_visual_init( std::string msg_frame_id )
{
  double x_max_dis = max_dis_ * x_max_dis_gain_;
  double y_max_dis = max_dis_ * y_max_dis_gain_;

  fov_node.resize(5);
  fov_node[0][0] = 0;
  fov_node[0][1] = 0;
  fov_node[0][2] = 0;

  fov_node[1][2] =  x_max_dis;
  fov_node[1][1] =  y_max_dis;
  fov_node[1][0] =  max_dis_;

  fov_node[2][2] =  x_max_dis;
  fov_node[2][1] = -y_max_dis;
  fov_node[2][0] =  max_dis_;

  fov_node[3][2] = -x_max_dis;
  fov_node[3][1] = -y_max_dis;
  fov_node[3][0] =  max_dis_;

  fov_node[4][2] = -x_max_dis;
  fov_node[4][1] =  y_max_dis;
  fov_node[4][0] =  max_dis_;

  markerNode_fov.header.frame_id = msg_frame_id;
  // markerNode_fov.header.stamp = msg_time;
  markerNode_fov.action = visualization_msgs::Marker::ADD;
  markerNode_fov.type = visualization_msgs::Marker::SPHERE_LIST;
  markerNode_fov.ns = "fov_nodes";
  // markerNode_fov.id = 0;
  markerNode_fov.pose.orientation.w = 1;
  markerNode_fov.scale.x = 0.05; markerNode_fov.scale.y = 0.05; markerNode_fov.scale.z = 0.05; 
  markerNode_fov.color.r = 0; markerNode_fov.color.g = 0.8; markerNode_fov.color.b = 1;
  markerNode_fov.color.a = 1;

  markerEdge_fov.header.frame_id = msg_frame_id;
  // markerEdge_fov.header.stamp = msg_time;
  markerEdge_fov.action = visualization_msgs::Marker::ADD;
  markerEdge_fov.type = visualization_msgs::Marker::LINE_LIST;
  markerEdge_fov.ns = "fov_edges";
  // markerEdge_fov.id = 0;
  markerEdge_fov.pose.orientation.w = 1;
  markerEdge_fov.scale.x = 0.05; 
  markerEdge_fov.color.r = 0.5f; markerEdge_fov.color.g = 0.0; markerEdge_fov.color.b = 0.0;
  markerEdge_fov.color.a = 1; 
}

void line_init( std::string msg_frame_id )
{
  marker_line.header.frame_id = msg_frame_id;
  marker_line.action = visualization_msgs::Marker::ADD;
  marker_line.type = visualization_msgs::Marker::LINE_LIST;
  marker_line.ns = "marker_line";
  marker_line.pose.orientation.w = 1;
  marker_line.scale.x = 0.05; 
  marker_line.color.r = 0.5f; marker_line.color.g = 0.5f; marker_line.color.b = 0.0;
  marker_line.color.a = 0.5; 

  fast_marker_line.header.frame_id = msg_frame_id;
  fast_marker_line.action = visualization_msgs::Marker::ADD;
  fast_marker_line.type = visualization_msgs::Marker::LINE_LIST;
  fast_marker_line.ns = "fast_marker_line";
  fast_marker_line.pose.orientation.w = 1;
  fast_marker_line.scale.x = 0.05; 
  fast_marker_line.color.r = 0.5f; fast_marker_line.color.g = 0.5f; fast_marker_line.color.b = 0.0;
  fast_marker_line.color.a = 0.5; 
}

void pub_line( Eigen::Vector3d& p1, Eigen::Vector3d& p2 )
{
  visualization_msgs::Marker clear_previous_msg;
  clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;

  visualization_msgs::MarkerArray markerArray_fov;
  geometry_msgs::Point pp1, pp2;
  pp1.x = p1[0];
  pp1.y = p1[1];
  pp1.z = p1[2];
  pp2.x = p2[0];
  pp2.y = p2[1];
  pp2.z = p2[2];

  marker_line.points.push_back(pp1);
  marker_line.points.push_back(pp2);

  markerArray_fov.markers.push_back(marker_line);
  line_pub_.publish(markerArray_fov);
}

void fast_pub_line( Eigen::Vector3d& p1, Eigen::Vector3d& p2 )
{
  visualization_msgs::Marker clear_previous_msg;
  clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;

  visualization_msgs::MarkerArray markerArray_fov;
  geometry_msgs::Point pp1, pp2;
  pp1.x = p1[0];
  pp1.y = p1[1];
  pp1.z = p1[2];
  pp2.x = p2[0];
  pp2.y = p2[1];
  pp2.z = p2[2];

  fast_marker_line.points.push_back(pp1);
  fast_marker_line.points.push_back(pp2);

  markerArray_fov.markers.push_back(fast_marker_line);
  fasttracker_line_pub_.publish(markerArray_fov);
}


void pub_fov_visual( Eigen::Vector3d& p, Eigen::Quaterniond& q )
{
  visualization_msgs::Marker clear_previous_msg;
  clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;

  visualization_msgs::MarkerArray markerArray_fov;
  markerNode_fov.points.clear();
  markerEdge_fov.points.clear();

  std::vector<geometry_msgs::Point> fov_node_marker;
  for(int i = 0; i < (int)fov_node.size(); i++)
  {
    Eigen::Vector3d vector_temp;
    vector_temp = q * fov_node[i] + p;
    geometry_msgs::Point point_temp;
    point_temp.x = vector_temp[0];
    point_temp.y = vector_temp[1];
    point_temp.z = vector_temp[2];
    fov_node_marker.push_back(point_temp);
  }

  markerNode_fov.points.push_back(fov_node_marker[0]);
  markerNode_fov.points.push_back(fov_node_marker[1]);
  markerNode_fov.points.push_back(fov_node_marker[2]);
  markerNode_fov.points.push_back(fov_node_marker[3]);
  markerNode_fov.points.push_back(fov_node_marker[4]);
  
  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[2]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[3]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerEdge_fov.points.push_back(fov_node_marker[1]);
  markerEdge_fov.points.push_back(fov_node_marker[2]);

  markerEdge_fov.points.push_back(fov_node_marker[2]);
  markerEdge_fov.points.push_back(fov_node_marker[3]);

  markerEdge_fov.points.push_back(fov_node_marker[3]);
  markerEdge_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[4]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerArray_fov.markers.push_back(clear_previous_msg);
  markerArray_fov.markers.push_back(markerNode_fov);
  markerArray_fov.markers.push_back(markerEdge_fov);
  fov_pub_.publish(markerArray_fov);
}

void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  // yaw traj
  have_yaw_ = msg->have_yaw;
  if( have_yaw_ )
  {
    Eigen::MatrixXd yaw_pts(3, msg->yaw_pts.size());
    for (int i = 0; i < msg->yaw_pts.size(); ++i) 
    {
      yaw_pts(0, i) = msg->yaw_pts[i];
      yaw_pts(1, i) = 0;
      yaw_pts(2, i) = 0;
    }
    UniformBspline yaw_traj(yaw_pts, 1, 0.1);
    yaw_traj.setKnot(knots);
    traj_.push_back(yaw_traj);
    traj_.push_back(traj_[3].getDerivative());
  }

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  // constexpr double YAW_DOT_MAX_PER_SEC = PI;
  constexpr double YAW_DOT_MAX_PER_SEC = 1.5;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw zx***/
    if( !have_yaw_ )
      yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    else
    {
      /*** get yaw wxx***/
      yaw_yawdot.first  = traj_[3].evaluateDeBoorT(t_cur)[0];
      yaw_yawdot.second = traj_[4].evaluateDeBoorT(t_cur)[0];
    }
    
    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first  = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  Eigen::Vector3d odom_pos{msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};
  Eigen::Quaterniond odom_q{msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z};

  pub_fov_visual(odom_pos, odom_q);

  if( drone_id_ == 0 )
  {
    static tf::TransformBroadcaster br;

    // 根据乌龟当前的位姿，设置相对于世界坐标系的坐标变换
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
    tf::Quaternion q{msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w};
    transform.setRotation(q);

    // 发布坐标变换
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "drone_tracker"));

    static int count_line = 0;
    count_line++;

    if(target_odom_ && (count_line > 10))
    {
      count_line = 0;
      pub_line(odom_pos, target_odom_pos_);

    }


    if(target_odom_)
    {
      Eigen::Vector3d re_pos = odom_q.inverse() * (target_odom_pos_ - odom_pos);
      std::ofstream out("/home/dwyane/benckmark/wxx_benchmark_sim/bag/AO/wxx.txt",ios::app);
      out<<re_pos(0)<<" "<<re_pos(1)<<" "<<re_pos(2)<<endl;
      out.close();
    }
  }
}

void fastTrackerOdomCallbck(const nav_msgs::Odometry& msg) {
  Eigen::Vector3d odom_pos{msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};
  Eigen::Quaterniond odom_q{msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z};


  if( drone_id_ == 0 )
  {
    // tf
    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) );
    tf::Quaternion q{msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w};
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "fast_tracker"));

    static int count_line_fasttracker = 0;
    count_line_fasttracker++;

    if(target_odom_ && (count_line_fasttracker > 100))
    {
      count_line_fasttracker = 0;
      fast_pub_line(odom_pos, target_odom_pos_);
    }

    static int txt_count_line_fasttracker = 0;
    txt_count_line_fasttracker++;

    if(target_odom_ && (txt_count_line_fasttracker > 10))
    {
      txt_count_line_fasttracker = 0;
      Eigen::Vector3d re_pos = odom_q.inverse() * (target_odom_pos_ - odom_pos);
      std::ofstream out("/home/dwyane/benckmark/wxx_benchmark_sim/bag/AO/fast_tracker.txt",ios::app);
      out<<re_pos(0)<<" "<<re_pos(1)<<" "<<re_pos(2)<<endl;
      out.close();
    }

  }
}


void targetOdomCallbck(const nav_msgs::Odometry& msg) {
  target_odom_pos_ = Eigen::Vector3d{msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};

  target_odom_ = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  nh.param("drone_id", drone_id_, -1);
  cout << drone_id_ << endl;
  cout << drone_id_ << endl;
  cout << drone_id_ << endl;
  cout << drone_id_ << endl;
  cout << drone_id_ << endl;
  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber odom_sub = nh.subscribe("/drone_0_visual_slam/odom", 50, odomCallbck, ros::TransportHints().tcpNoDelay());
  ros::Subscriber fasttracker_odom_sub = nh.subscribe("/visual_slam/odom", 50, fastTrackerOdomCallbck, ros::TransportHints().tcpNoDelay());

  // 给benchmark可视化用的
  ros::Subscriber target_odom_sub;
  if( drone_id_ == 0 )
  {
    target_odom_sub = nh.subscribe("/drone_1_visual_slam/odom", 50, targetOdomCallbck, ros::TransportHints().tcpNoDelay());
  }

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  fov_pub_ = nh.advertise<visualization_msgs::MarkerArray>("fov_visual", 5);	
  line_pub_ = nh.advertise<visualization_msgs::MarkerArray>("line_visual", 5);	
  fasttracker_line_pub_ = nh.advertise<visualization_msgs::MarkerArray>("fasttracker_line_visual", 5);	
  fasttracker_path_pub_ = nh.advertise<nav_msgs::Path>("fasttracker_path", 5);	

  fov_visual_init("world");
  line_init("world");

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}