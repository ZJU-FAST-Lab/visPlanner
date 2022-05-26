#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"
#include "bspline_opt/lbfgs_lite.hpp"

#include <traj_utils/plan_container.hpp>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace ego_planner
{

  class ControlPoints
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }

    void segment(ControlPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.clearance = clearance;
      buf.size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];

        // if ( buf.base_point[i - start].size() > 1 )
        // {
        //   ROS_ERROR("buf.base_point[i - start].size()=%d, base_point[i].size()=%d", buf.base_point[i - start].size(), base_point[i].size());
        // }
      }

      // cout << "RichInfoOneSeg_temp, insede" << endl;
      // for ( int k=0; k<buf.size; k++ )
      //   if ( buf.base_point[k].size() > 0 )
      //   {
      //     cout << "###" << buf.points.col(k).transpose() << endl;
      //     for (int k2 = 0; k2 < buf.base_point[k].size(); k2++)
      //     {
      //       cout << "      " << buf.base_point[k][k2].transpose() << " @ " << buf.direction[k][k2].transpose() << endl;
      //     }
      //   }
    }
  };

  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* main API */
    void setEnvironment(const GridMap::Ptr &map);
    void setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr mov_obj);
    void setParam(ros::NodeHandle &nh);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setInitYaw(double init_yaw);
    void setVelYaw(double target_vel_yaw);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);
    void setPosControlPointsAndTs(Eigen::MatrixXd &ctrl_pts, double ts);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most
    void setLocalTargetPt(const Eigen::Vector3d local_target_pt) { local_target_pt_ = local_target_pt; };

    void optimize();

    ControlPoints getControlPoints() { return cps_; };

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;

    std::vector<ControlPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);
    std::vector<std::pair<int, int>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts);
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }
    inline double getSwarmClearance(void) { return swarm_clearance_; }
    inline double getBsplineInterval(void) {return bspline_interval_;}


    // visibility
    void initControlPoints_yaw(const Eigen::MatrixXd &yaw_control_points);
    void setWaypoints_yaw(const vector<double> &waypts, const vector<int> &waypt_idx);
    bool BsplineOptimizeTrajReboundVisibility(Eigen::MatrixXd &optimal_points, double ts) ;
    bool BsplineOptimizeTrajReboundYaw(Eigen::MatrixXd &optimal_points, Eigen::MatrixXd &optimal_points_yaw, double ts, double init_yaw) ;
    
    vector<int> visibility_index, safe_yaw_index;
    vector<Eigen::Vector3d> visibility_target_point, visibility_gradient;
    vector<int> visibility_type;
    vector<double> visibility_coef;
    
    vector<Eigen::Vector3d> vis_pk, vis_pk_grad, vis_pk_grad_real;

  private:
    GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr moving_objs_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    int drone_id_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_; // B-spline knot span
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting

    double tracking_lambda_visibility_;
    double tracking_lambda_smooth_;
    double tracking_lambda_esdf_;
    double tracking_lambda_feasibility_;
    double tracking_lambda_tracking_dist_;
    double tracking_lambda_smoothness_yaw_;
    double tracking_lambda_feasibility_yaw_;
    double tracking_lambda_tracking_yaw_and_pos_;
    double tracking_lambda_safe_yaw_;


    int a;
    //
    double dist0_, swarm_clearance_; // safe distance
    double max_vel_, max_acc_;       // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    double t_now_for_swarm_;

    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    Eigen::Vector3d local_target_pt_; 

#define INIT_min_ellip_dist_ 123456789.0123456789
    double min_ellip_dist_;

    ControlPoints cps_, cps_yaw_;
    vector<double> waypoints_yaw_; // waypts constraints
    vector<int> waypt_idx_yaw_;             // waypts constraints index

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcMovingObjCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcSwarmCost_attract(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcESDFReboundCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcTrackingDistanceCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);

    bool check_collision_and_rebound(void);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize(double &final_cost);
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    // visibility
    bool rebound_optimize_visibility();
    static double costFunctionReboundVisibility(void *func_data, const double *x, double *grad, const int n);
    void combineCostReboundVisibility(const double *x, double *grad, double &f_combine, const int n);
    void calcVisibilityReboundCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);

    // yaw
    bool rebound_optimize_yaw();
    static double costFunctionReboundYaw(void *func_data, const double *x, double *grad, const int n);
    void combineCostReboundYaw(const double *x, double *grad, double &f_combine, const int n);
    void calcSmoothnessCost_yaw(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcSafeCost_yaw(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcFeasibilityCost_yaw(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcSwarmCost_posAndYaw_attract(const Eigen::MatrixXd &q_pos,  const Eigen::MatrixXd &q_yaw,
                                         double &cost_pos,              double &cost_yaw, 
                                         Eigen::MatrixXd &gradient_pos, Eigen::MatrixXd &gradient_yaw, Eigen::MatrixXd &gradient_pos_by_yaw);
    void calcTrackingYawAndPosCost(const Eigen::MatrixXd &q_pos,  const Eigen::MatrixXd &q_yaw,
                                   double &cost, 
                                   Eigen::MatrixXd &gradient_yaw, Eigen::MatrixXd &gradient_pos);
    
    double init_yaw_, init_yaw_sin_, init_yaw_cos_, target_vel_yaw_;
 
    vector<int> attract_pts_;             
    vector<Eigen::Vector3d> attract_pts_cor_drone_;             
    vector<double> attract_pts_best_yaw_;             

    bool if_equal( double x, double y );

    double best_attract_max_dist_, best_attract_min_dist_, attract_max_dist_threshold_, attract_min_dist_threshold_;


    /* for benckmark evaluation only */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner
#endif