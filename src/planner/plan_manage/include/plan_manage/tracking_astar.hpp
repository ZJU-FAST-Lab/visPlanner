#pragma once

#include <plan_env/grid_map.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <bspline_opt/uniform_bspline.h>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>

namespace ego_planner {

enum { REACH_HORIZON = 1, NO_PATH = 2, TOO_LONG = 3 };

static constexpr char IN_CLOSE_SET = 'a';
static constexpr char IN_OPEN_SET = 'b';
static constexpr char NOT_EXPAND = 'c';
static constexpr double inf = 1 >> 30;

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;
class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class KinodynamicAstar {
 private:
  UniformBspline tracked_traj_;
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  GridMap::Ptr gridMapPtr_;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt) {
    return ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  }
  int timeToIndex(double time) {
    return floor((time - time_origin_) * inv_time_resolution_);
  }
  void retrievePath(PathNodePtr end_node) {
    PathNodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL) {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
  }

  /* shot trajectory */
  vector<double> cubic(double a, double b, double c, double d) {
    vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3 * a1 - a2 * a2) / 9;
    double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
    double D = Q * Q * Q + R * R;
    if (D > 0) {
      double S = std::cbrt(R + sqrt(D));
      double T = std::cbrt(R - sqrt(D));
      dts.push_back(-a2 / 3 + (S + T));
      return dts;
    } else if (D == 0) {
      double S = std::cbrt(R);
      dts.push_back(-a2 / 3 + S + S);
      dts.push_back(-a2 / 3 - S);
      return dts;
    } else {
      double theta = acos(R / sqrt(-Q * Q * Q));
      dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
      dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
      dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
      return dts;
    }
  }
  vector<double> quartic(double a, double b, double c, double d, double e) {
    vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    vector<double> ys =
        cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
    double y1 = ys.front();
    double r = a3 * a3 / 4 - a2 + y1;
    if (r < 0)
      return dts;

    double R = sqrt(r);
    double D, E;
    if (R != 0) {
      D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 +
               0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
      E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 -
               0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    } else {
      D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
      E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
    }

    if (!std::isnan(D)) {
      dts.push_back(-a3 / 4 + R / 2 + D / 2);
      dts.push_back(-a3 / 4 + R / 2 - D / 2);
    }
    if (!std::isnan(E)) {
      dts.push_back(-a3 / 4 - R / 2 + E / 2);
      dts.push_back(-a3 / 4 - R / 2 - E / 2);
    }

    return dts;
  }

  double estimateHeuristic(Eigen::VectorXd x1,
                           Eigen::VectorXd x2,
                           double& optimal_time) {
    const Eigen::Vector3d dp = x2.head(3) - x1.head(3);
    const Eigen::Vector3d v0 = x1.segment(3, 3);
    const Eigen::Vector3d v1 = x2.segment(3, 3);

    double c1 = -36 * dp.dot(dp);
    double c2 = 24 * (v0 + v1).dot(dp);
    double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c4 = 0;
    double c5 = w_time_;

    std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

    double v_max = max_vel_ * 0.5;
    double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Eigen::Infinity>() / v_max;
    ts.push_back(t_bar);

    double cost = 100000000;
    double t_d = t_bar;

    for (auto t : ts) {
      if (t < t_bar)
        continue;
      double c =
          -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
      if (c < cost) {
        cost = c;
        t_d = t;
      }
    }

    optimal_time = t_d;

    return 1.0 * (1 + tie_breaker_) * cost;
  }

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                    Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d um,
                    double tau) {
    for (int i = 0; i < 3; ++i)
      phi_(i, i + 3) = tau;

    Eigen::Matrix<double, 6, 1> integral;
    integral.head(3) = 0.5 * pow(tau, 2) * um;
    integral.tail(3) = tau * um;

    state1 = phi_ * state0 + integral;
  }

  bool rayValid(const Eigen::Vector3d& p1,
                const Eigen::Vector3d& v,
                const double t) {
    if (t > tracked_traj_.getTimeSum()) {
      return !gridMapPtr_->getInflateOccupancy(p1);
    }
    Eigen::Vector3d p2 = tracked_traj_.evaluateDeBoorT(t);
    double dist = (p1 - p2).norm();

    int n = floor(dist / resolution_);
    for (int i = 0; i < n; ++i) {
      double lambda = 1.0 * i / n;
      Eigen::Vector3d p = (1 - lambda) * p1 + lambda * p2;
      if (gridMapPtr_->getInflateOccupancy(p)) {
        return false;
      }
    }
    return true;
  }

 public:
  KinodynamicAstar(ros::NodeHandle& nh, const GridMap::Ptr& env) {
    nh.param("search/max_tau", max_tau_, -1.0);
    nh.param("search/init_max_tau", init_max_tau_, -1.0);
    nh.param("search/max_vel", max_vel_, -1.0);
    nh.param("search/max_acc", max_acc_, -1.0);
    nh.param("search/w_time", w_time_, -1.0);
    nh.param("search/horizon", horizon_, -1.0);
    nh.param("search/resolution_astar", resolution_, -1.0);
    nh.param("search/time_resolution", time_resolution_, -1.0);
    nh.param("search/lambda_heu", lambda_heu_, -1.0);
    nh.param("search/allocate_num", allocate_num_, -1);
    nh.param("search/check_num", check_num_, -1);
    nh.param("search/optimistic", optimistic_, true);
    tie_breaker_ = 1.0 + 1.0 / 10000;

    double vel_margin;
    nh.param("search/vel_margin", vel_margin, 0.0);
    max_vel_ += vel_margin;
    max_vel_ *= 0.8;
    max_acc_ *= 0.8;

    this->gridMapPtr_ = env;

    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    inv_time_resolution_ = 1.0 / time_resolution_;
    gridMapPtr_->getRegion(origin_, map_size_3d_);

    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_3d_.transpose() << endl;

    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++) {
      path_node_pool_[i] = new PathNode;
    }

    phi_ = Eigen::MatrixXd::Identity(6, 6);
    use_node_num_ = 0;
    iter_num_ = 0;
  }
  ~KinodynamicAstar() {
    for (int i = 0; i < allocate_num_; i++) {
      delete path_node_pool_[i];
    }
  }

  /* main API */
  void reset() {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
        empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++) {
      PathNodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
    has_path_ = false;
  }
  int search(const UniformBspline& traj,
             const Eigen::Vector3d& start_pt,
             const Eigen::Vector3d& start_v,
             const Eigen::Vector3d& start_a,
             bool init,
             bool dynamic = true,
             double time_start = 0.0) {
    ros::Time t_jlji = ros::Time::now();
    tracked_traj_ = traj;
    Eigen::Vector3d end_pt =
        tracked_traj_.evaluateDeBoorT(tracked_traj_.getTimeSum());
    Eigen::Vector3d end_v = tracked_traj_.getDerivative().evaluateDeBoorT(
        tracked_traj_.getTimeSum());
    start_vel_ = start_v;
    start_acc_ = start_a;

    PathNodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->state.head(3) = start_pt;
    cur_node->state.tail(3) = start_v;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_score = 0.0;

    Eigen::VectorXd end_state(6);
    Eigen::Vector3i end_index;
    double time_to_goal;

    end_state.head(3) = end_pt;
    end_state.tail(3) = end_v;
    end_index = posToIndex(end_pt);
    cur_node->f_score =
        lambda_heu_ *
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
    cur_node->node_state = IN_OPEN_SET;
    open_set_.push(cur_node);
    use_node_num_ += 1;

    if (dynamic) {
      time_origin_ = time_start;
      cur_node->time = time_start;
      cur_node->time_idx = timeToIndex(time_start);
      expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
      // cout << "time start: " << time_start << endl;
    } else
      expanded_nodes_.insert(cur_node->index, cur_node);

    PathNodePtr neighbor = NULL;
    PathNodePtr terminate_node = NULL;
    bool init_search = init;
    const int tolerance = ceil(1 / resolution_);

    while (!open_set_.empty()) {
      if( (ros::Time::now() - t_jlji).toSec() > 0.01 ){
        ROS_WARN("Search too long time!");
        return TOO_LONG;
      }
      cur_node = open_set_.top();
      // std::cout << "time: " << cur_node->time << std::endl;

      // Terminate?
      bool reach_horizon = cur_node->time >= tracked_traj_.getTimeSum() ||
                           (cur_node->state.head(3) - end_pt).norm() <= 2.0;

      if (reach_horizon) {
        terminate_node = cur_node;
        retrievePath(terminate_node);
        return REACH_HORIZON;
      }

      open_set_.pop();
      cur_node->node_state = IN_CLOSE_SET;
      iter_num_ += 1;

      double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
      Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
      Eigen::Matrix<double, 6, 1> pro_state;
      vector<PathNodePtr> tmp_expand_nodes;
      Eigen::Vector3d um;
      double pro_t;
      vector<Eigen::Vector3d> inputs;
      vector<double> durations;
      if (init_search) {
        inputs.push_back(start_acc_);
        for (double tau = time_res_init * init_max_tau_;
             tau <= init_max_tau_ + 1e-3; tau += time_res_init * init_max_tau_)
          durations.push_back(tau);
        init_search = false;
      } else {
        for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
          for (double ay = -max_acc_; ay <= max_acc_ + 1e-3;
               ay += max_acc_ * res)
            for (double az = -max_acc_; az <= max_acc_ + 1e-3;
                 az += max_acc_ * res) {
              um << ax, ay, az;
              inputs.push_back(um);
            }
        for (double tau = time_res * max_tau_; tau <= max_tau_;
             tau += time_res * max_tau_)
          durations.push_back(tau);
      }

      // cout << "cur state:" << cur_state.head(3).transpose() << endl;
      for (int i = 0; i < (int)inputs.size(); ++i)
        for (int j = 0; j < (int)durations.size(); ++j) {
          um = inputs[i];
          double tau = durations[j];
          stateTransit(cur_state, pro_state, um, tau);
          pro_t = cur_node->time + tau;

          Eigen::Vector3d pro_pos = pro_state.head(3);

          // Check if in close set
          Eigen::Vector3i pro_id = posToIndex(pro_pos);
          int pro_t_id = timeToIndex(pro_t);
          PathNodePtr pro_node = dynamic
                                     ? expanded_nodes_.find(pro_id, pro_t_id)
                                     : expanded_nodes_.find(pro_id);
          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
            if (init_search)
              std::cout << "close" << std::endl;
            continue;
          }

          // Check maximal velocity
          Eigen::Vector3d pro_v = pro_state.tail(3);
          if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ ||
              fabs(pro_v(2)) > max_vel_) {
            if (init_search)
              std::cout << "vel" << std::endl;
            continue;
          }

          // Check not in the same voxel
          Eigen::Vector3i diff = pro_id - cur_node->index;
          int diff_time = pro_t_id - cur_node->time_idx;
          if (diff.norm() == 0 && ((!dynamic) || diff_time == 0)) {
            if (init_search)
              std::cout << "same" << std::endl;
            continue;
          }

          // Check safety
          Eigen::Vector3d pos, vel;
          Eigen::Matrix<double, 6, 1> xt;
          bool is_occ = false;
          for (int k = 1; k <= check_num_; ++k) {
            double dt = tau * double(k) / double(check_num_);
            stateTransit(cur_state, xt, um, dt);
            pos = xt.head(3);
            vel = xt.tail(3);
            // TODO check cur_node->time + dt
            // if (gridMapPtr_->getInflateOccupancy(pos) == 1) {
            if (!rayValid(pos, vel, cur_node->time + dt)) {
              is_occ = true;
              break;
            }
          }
          if (is_occ) {
            if (init_search)
              std::cout << "safe" << std::endl;
            continue;
          }

          double time_to_goal, tmp_g_score, tmp_f_score;
          tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
          tmp_f_score = tmp_g_score +
                        lambda_heu_ * estimateHeuristic(pro_state, end_state,
                                                        time_to_goal);

          // Compare nodes expanded from the same parent
          bool prune = false;
          for (int j = 0; j < (int)tmp_expand_nodes.size(); ++j) {
            PathNodePtr expand_node = tmp_expand_nodes[j];
            if ((pro_id - expand_node->index).norm() == 0 &&
                ((!dynamic) || pro_t_id == expand_node->time_idx)) {
              prune = true;
              if (tmp_f_score < expand_node->f_score) {
                expand_node->f_score = tmp_f_score;
                expand_node->g_score = tmp_g_score;
                expand_node->state = pro_state;
                expand_node->input = um;
                expand_node->duration = tau;
                if (dynamic)
                  expand_node->time = cur_node->time + tau;
              }
              break;
            }
          }

          // This node end up in a voxel different from others
          if (!prune) {
            if (pro_node == NULL) {
              pro_node = path_node_pool_[use_node_num_];
              pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              pro_node->node_state = IN_OPEN_SET;
              if (dynamic) {
                pro_node->time = cur_node->time + tau;
                pro_node->time_idx = timeToIndex(pro_node->time);
              }
              open_set_.push(pro_node);

              if (dynamic)
                expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
              else
                expanded_nodes_.insert(pro_id, pro_node);

              tmp_expand_nodes.push_back(pro_node);

              use_node_num_ += 1;
              if (use_node_num_ == allocate_num_) {
                cout << "run out of memory." << endl;
                return NO_PATH;
              }
            } else if (pro_node->node_state == IN_OPEN_SET) {
              if (tmp_g_score < pro_node->g_score) {
                // pro_node->index = pro_id;
                pro_node->state = pro_state;
                pro_node->f_score = tmp_f_score;
                pro_node->g_score = tmp_g_score;
                pro_node->input = um;
                pro_node->duration = tau;
                pro_node->parent = cur_node;
                if (dynamic)
                  pro_node->time = cur_node->time + tau;
              }
            } else {
              cout << "error type in searching: " << pro_node->node_state
                   << endl;
            }
          }
        }
      // init_search = false;
    }

    cout << "open set empty, no path!" << endl;
    cout << "use node num: " << use_node_num_ << endl;
    cout << "iter num: " << iter_num_ << endl;
    return NO_PATH;
  }

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t) {
    std::vector<Eigen::Vector3d> state_list;

    /* ---------- get traj of searching ---------- */
    PathNodePtr node = path_nodes_.back();
    Eigen::Matrix<double, 6, 1> x0, xt;

    while (node->parent != NULL) {
      Eigen::Vector3d ut = node->input;
      double duration = node->duration;
      x0 = node->parent->state;

      for (double t = duration; t >= -1e-5; t -= delta_t) {
        stateTransit(x0, xt, ut, t);
        state_list.push_back(xt.head(3));
      }
      node = node->parent;
    }
    reverse(state_list.begin(), state_list.end());
    /* ---------- get traj of one shot ---------- */
    if (is_shot_succ_) {
      Eigen::Vector3d coord;
      Eigen::VectorXd poly1d, time(4);

      for (double t = delta_t; t <= t_shot_; t += delta_t) {
        for (int j = 0; j < 4; j++)
          time(j) = pow(t, j);

        for (int dim = 0; dim < 3; dim++) {
          poly1d = coef_shot_.row(dim);
          coord(dim) = poly1d.dot(time);
        }
        state_list.push_back(coord);
      }
    }

    return state_list;
  }

  inline double getT() {
    double T_sum = 0.0;
    PathNodePtr node = path_nodes_.back();
    while (node->parent != NULL) {
      T_sum += node->duration;
      node = node->parent;
    }
    return T_sum;
  }

  void getSamples(double& ts,
                  vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives) {
    /* ---------- path duration ---------- */
    double T_sum = 0.0;
    PathNodePtr node = path_nodes_.back();
    while (node->parent != NULL) {
      T_sum += node->duration;
      node = node->parent;
    }
    // cout << "duration:" << T_sum << endl;

    // Calculate boundary vel and acc
    Eigen::Vector3d end_vel, end_acc;
    double t;
    if (is_shot_succ_) {
      t = t_shot_;
      end_vel = end_vel_;
      for (int dim = 0; dim < 3; ++dim) {
        Eigen::Vector4d coe = coef_shot_.row(dim);
        end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
      }
    } else {
      t = path_nodes_.back()->duration;
      end_vel = node->state.tail(3);
      end_acc = path_nodes_.back()->input;
    }

    // Get point samples
    int seg_num = floor(T_sum / ts);
    seg_num = max(8, seg_num);
    ts = T_sum / double(seg_num);
    bool sample_shot_traj = is_shot_succ_;
    node = path_nodes_.back();

    for (double ti = T_sum; ti > -1e-5; ti -= ts) {
      if (sample_shot_traj) {
        // samples on shot traj
        Eigen::Vector3d coord;
        Eigen::Vector4d poly1d, time;

        for (int j = 0; j < 4; j++)
          time(j) = pow(t, j);

        for (int dim = 0; dim < 3; dim++) {
          poly1d = coef_shot_.row(dim);
          coord(dim) = poly1d.dot(time);
        }

        point_set.push_back(coord);
        t -= ts;

        /* end of segment */
        if (t < -1e-5) {
          sample_shot_traj = false;
          if (node->parent != NULL)
            t += node->duration;
        }
      } else {
        // samples on searched traj
        Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
        Eigen::Matrix<double, 6, 1> xt;
        Eigen::Vector3d ut = node->input;

        stateTransit(x0, xt, ut, t);

        point_set.push_back(xt.head(3));
        t -= ts;

        // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
        if (t < -1e-5 && node->parent->parent != NULL) {
          node = node->parent;
          t += node->duration;
        }
      }
    }
    reverse(point_set.begin(), point_set.end());

    // calculate start acc
    Eigen::Vector3d start_acc;
    if (path_nodes_.back()->parent == NULL) {
      // no searched traj, calculate by shot traj
      start_acc = 2 * coef_shot_.col(2);
    } else {
      // input of searched traj
      start_acc = node->input;
    }

    start_end_derivatives.push_back(start_vel_);
    start_end_derivatives.push_back(end_vel);
    start_end_derivatives.push_back(start_acc);
    start_end_derivatives.push_back(end_acc);
  }

  std::vector<PathNodePtr> getVisitedNodes() {
    vector<PathNodePtr> visited;
    visited.assign(path_node_pool_.begin(),
                   path_node_pool_.begin() + use_node_num_ - 1);
    return visited;
  }

  inline bool search(const UniformBspline& traj,
                    const Eigen::MatrixXd& iniState,
                    double time_offset = 0.0) {
    cout << "[kino replan]: kinodynamic search begin!" << endl;

    reset();
    int status = search(traj, iniState.col(0), iniState.col(1), iniState.col(2),
                        true, true, time_offset);

    if (status == NO_PATH) {
      cout << "[kino replan]: kinodynamic search fail!" << endl;

      // retry searching with discontinuous initial state
      reset();
      status = search(traj, iniState.col(0), iniState.col(1), iniState.col(2),
                      false, true, time_offset);

      if (status == NO_PATH) {
        cout << "[kino replan]: Can't find path." << endl;
      } else {
        cout << "[kino replan]: retry search success." << endl;
      }
    } else if (status == REACH_HORIZON)
    {
      cout << "[kino replan]: kinodynamic search success." << endl;
    }
    return (status == REACH_HORIZON);
  }

  typedef shared_ptr<KinodynamicAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ego_planner