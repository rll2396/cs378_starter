//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Vertex {
    std::string id;
    Eigen::Vector2f loc;
    std::vector<std::string> neighbors;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, const double dist, const double curv, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();

  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

 private:

  float safety_margin = 0.1;
  float w = 0.14 + safety_margin;
  float h = 0.43 + safety_margin;

  // Map of the environment.
  vector_map::VectorMap map_;
  // Last seen point cloud
  std::vector<Eigen::Vector2f> point_cloud;
  // To account for initial values being 0
  bool initialized;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Graph of map.
  std::map<std::string, Vertex> graph;
  std::vector<Eigen::Vector2f> planned_path;
  std::string goal_vertex_id;
  int runs_since_path_calc;
  bool nav_goal_set_;

  bool TooCloseToWall(const Eigen::Vector2f vertex_loc);
  std::string GetClosestVertexID(const Eigen::Vector2f point);
  Eigen::Vector2f GlobalizePoint(const Eigen::Vector2f& local_point);
  Eigen::Vector2f LocalizePoint(const Eigen::Vector2f& global_point);
  void DrawCar(const Eigen::Vector2f& local_point, uint32_t color, float angle);
  void MakeGraph();
  void CalculatePath();
};

}  // namespace navigation

#endif  // NAVIGATION_H
