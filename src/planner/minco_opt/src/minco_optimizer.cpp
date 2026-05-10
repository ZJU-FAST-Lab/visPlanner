#include <minco_opt/minco_optimizer.h>

#include <algorithm>
#include <cmath>

namespace ego_planner {
namespace {

inline Eigen::Matrix<double, 3, 1> solveHighOrderCoeff(double p0, double v0, double a0,
                                                        double pT, double vT, double T) {
  // quintic: p(t)=c0+c1 t+c2 t^2+c3 t^3+c4 t^4+c5 t^5
  // c0,c1,c2 fixed by start states; solve c3,c4,c5 by terminal p/v/a(=0)
  const double c0 = p0;
  const double c1 = v0;
  const double c2 = 0.5 * a0;

  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  Eigen::Matrix3d A;
  A << T3, T4, T5,
       3.0 * T2, 4.0 * T3, 5.0 * T4,
       6.0 * T, 12.0 * T2, 20.0 * T3;

  Eigen::Vector3d b;
  b << pT - (c0 + c1 * T + c2 * T2),
       vT - (c1 + 2.0 * c2 * T),
       -2.0 * c2; // terminal acceleration set to zero

  return A.colPivHouseholderQr().solve(b);
}

} // namespace

void MincoOptimizer::setLimits(double max_vel, double max_acc) {
  max_vel_ = max_vel;
  max_acc_ = max_acc;
}

bool MincoOptimizer::optimize(const Eigen::Vector3d& start_p,
                              const Eigen::Vector3d& start_v,
                              const Eigen::Vector3d& start_a,
                              const Eigen::Vector3d& goal_p,
                              const Eigen::Vector3d& goal_v,
                              MincoResult& out) {
  const double distance = (goal_p - start_p).norm();
  const double vel = std::max(0.2, max_vel_);
  const double acc = std::max(0.2, max_acc_);

  // Conservative time heuristic to keep trajectory feasible.
  const double t_by_v = distance / vel;
  const double t_by_a = std::sqrt(distance / acc);
  const double duration = std::max(0.5, std::max(1.2 * t_by_v, 2.0 * t_by_a));

  out.coeff_x = Eigen::MatrixXd::Zero(1, 6);
  out.coeff_y = Eigen::MatrixXd::Zero(1, 6);
  out.coeff_z = Eigen::MatrixXd::Zero(1, 6);
  out.durations = Eigen::VectorXd::Constant(1, duration);

  for (int axis = 0; axis < 3; ++axis) {
    const double p0 = start_p(axis);
    const double v0 = start_v(axis);
    const double a0 = start_a(axis);
    const double pT = goal_p(axis);
    const double vT = goal_v(axis);

    const auto c345 = solveHighOrderCoeff(p0, v0, a0, pT, vT, duration);

    Eigen::RowVectorXd coeff(6);
    coeff << p0, v0, 0.5 * a0, c345(0), c345(1), c345(2); // ascending powers

    if (axis == 0)
      out.coeff_x.row(0) = coeff;
    else if (axis == 1)
      out.coeff_y.row(0) = coeff;
    else
      out.coeff_z.row(0) = coeff;
  }

  return true;
}

} // namespace ego_planner
