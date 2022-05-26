#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace ego_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;

    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);

    arrow_ids_.clear();
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      //if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    //if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
  }

  void PlanningVisualization::displaySphereList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
    }
    pub.publish(sphere);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  // real ids used: {id ~ (arrow nums)+id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    // clear
    visualization_msgs::Marker clear_all;
    clear_all.action = visualization_msgs::Marker::DELETEALL;
    clear_all.id = 0;
    array.markers.push_back(clear_all);

    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;
    arrow.pose.orientation.w = 1.0;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      // arrow.id = i + id * 1000;
      arrow.id++;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }

  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    static int last_nums = 0;

    for ( int id=0; id<last_nums; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(init_list_pub, blank, scale, color, id, false);
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<init_trajs.size(); id++ )
    {
      Eigen::Vector4d color(0, 0, 1, 0.7);
      displayMarkerList(init_list_pub, init_trajs[id], scale, color, id, false);
      ros::Duration(0.001).sleep();
      last_nums++;
    }

  }

  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 1, 1);
    displaySphereList(optimal_list_pub, list, 0.15, color, id);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths)
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;

    // new
    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }

  // void PlanningVisualization::displayTrajWithYaw( Eigen::VectorXd &knots, Eigen::MatrixXd &pos_ptsint, Eigen::MatrixXd &yaw_pts,
  //                                                 double start_time, double duration, int id1, int id2 )
  // {
  //   // pos traj
  //   UniformBspline pos_traj(pos_pts, 3, 0.1);
  //   pos_traj.setKnot(knots);

  //   // yaw traj
  //   UniformBspline yaw_traj(yaw_pts, 3, 0.1);
  //   yaw_traj.setKnot(knots);

  //   double display_dt = 0.01;

  //   vector<Eigen::Vector3d> pos_list;
  //   vector<Eigen::Vector3d> yaw_list;
  //   vector<Eigen::Vector3d> cps_list;

  //   Eigen::Vector3d pos(Eigen::Vector3d::Zero()), yaw_pos(Eigen::Vector3d::Zero()), cpt_pos(Eigen::Vector3d::Zero());

  //   // pos waypoints
  //   for( double t = 0; t < duration; t = t + display_dt )
  //   {
  //     double t_now = t + start_time;
  //     pos = pos_traj.evaluateDeBoorT(t_now);
  //     pos_list.push_back(pos);

  //     double yaw = yaw_traj.evaluateDeBoorT(t_now)[0];
  //     yaw_list.push_back(pos);
  //     yaw_pos = pos + Eigen::Vector3d{ cos(yaw), sin(yaw), 0};
  //     yaw_list.push_back(yaw_pos);
  //   }
  //   Eigen::Vector4d color1(1, 0, 0, 1);
  //   Eigen::Vector4d color2(0, 1, 0, 1);
  //   displayMarkerList(pos_list_pub, pos_list, 0.15, color1, id1);
  //   displayArrowList (yaw_list_pub, yaw_list, 0.5 , color2, id2);
  // }


  // PlanningVisualization::
} // namespace ego_planner