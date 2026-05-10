#include <ros/ros.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/Bspline.h>

namespace {

using ego_planner::UniformBspline;

struct TrajHolder {
  bool ready{false};
  int64_t traj_id{-1};
  ros::Time start_time;
  UniformBspline position;

  TrajHolder() : position(Eigen::MatrixXd::Zero(3, 6), 3, 0.1) {}
};

class TrajCompareNode {
public:
  explicit TrajCompareNode(ros::NodeHandle &nh) : nh_(nh) {
    nh_.param("compare/topic_a", topic_a_, std::string("/drone_0_planning/bspline_minco"));
    nh_.param("compare/topic_b", topic_b_, std::string("/drone_0_planning/bspline_bspline"));
    nh_.param("compare/sample_dt", sample_dt_, 0.05);
    nh_.param("compare/enable_csv", enable_csv_, true);
    nh_.param("compare/csv_path", csv_path_, std::string("/tmp/traj_compare.csv"));

    sub_a_ = nh_.subscribe(topic_a_, 10, &TrajCompareNode::cbA, this, ros::TransportHints().tcpNoDelay());
    sub_b_ = nh_.subscribe(topic_b_, 10, &TrajCompareNode::cbB, this, ros::TransportHints().tcpNoDelay());
    timer_ = nh_.createTimer(ros::Duration(0.2), &TrajCompareNode::onTimer, this);

    if (enable_csv_) {
      initCsv();
    }

    ROS_INFO_STREAM("[traj_compare] listening topic_a=" << topic_a_ << ", topic_b=" << topic_b_ << ", dt=" << sample_dt_);
    if (enable_csv_) {
      ROS_INFO_STREAM("[traj_compare] csv logging enabled: " << csv_path_);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_a_, sub_b_;
  ros::Timer timer_;

  std::string topic_a_, topic_b_;
  double sample_dt_{0.05};
  bool enable_csv_{true};
  std::string csv_path_;

  TrajHolder a_, b_;
  int64_t last_compared_a_{std::numeric_limits<int64_t>::min()};
  int64_t last_compared_b_{std::numeric_limits<int64_t>::min()};

  static bool parseMsg(const traj_utils::BsplineConstPtr &msg, TrajHolder &out) {
    if (!msg || msg->pos_pts.empty() || msg->knots.size() < 2) {
      return false;
    }

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
      pos_pts(0, i) = msg->pos_pts[i].x;
      pos_pts(1, i) = msg->pos_pts[i].y;
      pos_pts(2, i) = msg->pos_pts[i].z;
    }

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) {
      knots(i) = msg->knots[i];
    }

    out.position = UniformBspline(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    out.position.setKnot(knots);
    out.traj_id = msg->traj_id;
    out.start_time = msg->start_time;
    out.ready = true;
    return true;
  }

  void cbA(const traj_utils::BsplineConstPtr &msg) {
    if (!parseMsg(msg, a_)) {
      ROS_WARN_THROTTLE(1.0, "[traj_compare] invalid message on topic A");
    }
  }

  void cbB(const traj_utils::BsplineConstPtr &msg) {
    if (!parseMsg(msg, b_)) {
      ROS_WARN_THROTTLE(1.0, "[traj_compare] invalid message on topic B");
    }
  }

  static double estimateLength(UniformBspline traj, double duration, double dt) {
    double len = 0.0;
    Eigen::Vector3d prev = traj.evaluateDeBoorT(0.0);
    for (double t = dt; t <= duration + 1e-6; t += dt) {
      const double tt = std::min(t, duration);
      Eigen::Vector3d cur = traj.evaluateDeBoorT(tt);
      len += (cur - prev).norm();
      prev = cur;
    }
    return len;
  }

  void initCsv() {
    std::ifstream in(csv_path_);
    const bool exists = in.good();
    in.close();

    if (!exists) {
      std::ofstream out(csv_path_, std::ios::out);
      out << "wall_time,topic_a,topic_b,traj_id_a,traj_id_b,start_time_a,start_time_b,duration,mean_dist,max_dist,end_dist,len_a,len_b,sample_dt\n";
      out.close();
    }
  }

  void appendCsv(double duration, double mean_dist, double max_dist, double end_dist, double len_a, double len_b) {
    if (!enable_csv_) {
      return;
    }
    std::ofstream out(csv_path_, std::ios::app);
    if (!out.is_open()) {
      ROS_WARN_THROTTLE(1.0, "[traj_compare] failed to open csv file: %s", csv_path_.c_str());
      return;
    }
    out << std::fixed << std::setprecision(6)
        << ros::Time::now().toSec() << ","
        << topic_a_ << ","
        << topic_b_ << ","
        << a_.traj_id << ","
        << b_.traj_id << ","
        << a_.start_time.toSec() << ","
        << b_.start_time.toSec() << ","
        << duration << ","
        << mean_dist << ","
        << max_dist << ","
        << end_dist << ","
        << len_a << ","
        << len_b << ","
        << sample_dt_ << "\n";
    out.close();
  }

  void onTimer(const ros::TimerEvent &) {
    if (!a_.ready || !b_.ready) {
      return;
    }
    if (a_.traj_id == last_compared_a_ && b_.traj_id == last_compared_b_) {
      return;
    }

    const double dur_a = a_.position.getTimeSum();
    const double dur_b = b_.position.getTimeSum();
    const double duration = std::min(dur_a, dur_b);

    if (duration <= sample_dt_) {
      ROS_WARN("[traj_compare] duration too short for comparison.");
      return;
    }

    double sum_dist = 0.0;
    double max_dist = 0.0;
    int n = 0;

    for (double t = 0.0; t <= duration + 1e-6; t += sample_dt_) {
      const double tt = std::min(t, duration);
      const Eigen::Vector3d pa = a_.position.evaluateDeBoorT(tt);
      const Eigen::Vector3d pb = b_.position.evaluateDeBoorT(tt);
      const double d = (pa - pb).norm();
      sum_dist += d;
      max_dist = std::max(max_dist, d);
      ++n;
    }

    const Eigen::Vector3d end_a = a_.position.evaluateDeBoorT(duration);
    const Eigen::Vector3d end_b = b_.position.evaluateDeBoorT(duration);
    const double end_dist = (end_a - end_b).norm();

    const double len_a = estimateLength(a_.position, duration, sample_dt_);
    const double len_b = estimateLength(b_.position, duration, sample_dt_);
    const double mean_dist = sum_dist / std::max(1, n);

    ROS_INFO_STREAM("[traj_compare] A(traj_id=" << a_.traj_id << ") vs B(traj_id=" << b_.traj_id
                    << ") | duration=" << duration
                    << " | mean_dist=" << mean_dist
                    << " | max_dist=" << max_dist
                    << " | end_dist=" << end_dist
                    << " | len_a=" << len_a
                    << " | len_b=" << len_b);

    appendCsv(duration, mean_dist, max_dist, end_dist, len_a, len_b);

    last_compared_a_ = a_.traj_id;
    last_compared_b_ = b_.traj_id;
  }
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_compare_node");
  ros::NodeHandle nh("~");
  TrajCompareNode node(nh);
  ros::spin();
  return 0;
}
