#pragma once

#include <Eigen/Eigen>
#include <memory>

namespace ego_planner {

struct MincoResult {
  Eigen::MatrixXd coeff_x;
  Eigen::MatrixXd coeff_y;
  Eigen::MatrixXd coeff_z;
  Eigen::VectorXd durations;
};

class MincoOptimizer {
public:
  using Ptr = std::shared_ptr<MincoOptimizer>;

  MincoOptimizer() = default;
  ~MincoOptimizer() = default;

  void setLimits(double max_vel, double max_acc);

  bool optimize(const Eigen::Vector3d& start_p,
                const Eigen::Vector3d& start_v,
                const Eigen::Vector3d& start_a,
                const Eigen::Vector3d& goal_p,
                const Eigen::Vector3d& goal_v,
                MincoResult& out);

private:
  double max_vel_{0.0};
  double max_acc_{0.0};
};

} // namespace ego_planner
