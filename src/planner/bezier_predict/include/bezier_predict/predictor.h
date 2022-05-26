#ifndef _PREDICTOR_H_
#define _PREDICTOR_H_

#include "bezier_predict.h"
#include <list>
#include <plan_env/grid_map.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/Bspline.h>
#include <bspline_opt/uniform_bspline.h>

namespace ego_planner
{
  class Predictor
  {

  public:
    Predictor() {}
    ~Predictor() {}

    /* main API */
    void setEnvironment(const GridMap::Ptr &map);
    void setParam(ros::NodeHandle &nh, PlanningVisualization::Ptr vis);
    bool generatePredict( std::vector<Eigen::Vector3d> &sample_list, std::vector<double> &sample_time_list_ros );
    void updateAndVisualPredict();

    std::vector<Eigen::Vector4d> target_list_;
    std::vector<Eigen::Vector3d> sample_list_;
    std::vector<double> sample_time_list_ros_;

    typedef std::shared_ptr<Predictor> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    GridMap::Ptr grid_map_;
    int drone_id_;
    ros::Subscriber odom_sub_, target_odom_sub_;
    ros::Publisher predict_list_pub_, broadcast_bspline_pub_;
    Eigen::Vector3d odom_now_;
    double yaw_now_;
    bool have_odom_;
    size_t list_max_size_;
    int count_;
    int freq_donw_;
    Bezierpredict bezierpredict_;
    PlanningVisualization::Ptr visualization_;
    ros::Time start_time_;

    // paramter
    double yaw_threshold_;
    double max_dis_;
    double min_dis_;
    
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void RealworldTargetOdometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void SimulatorTargetOdometryCallback(const nav_msgs::OdometryConstPtr &msg);
  };

} // namespace ego_planner
#endif