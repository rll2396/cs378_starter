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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "navigation/simple_queue.h"

using Eigen::Vector2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using f1tenth_course::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, const double dist, const double curv, ros::NodeHandle* n) :
    point_cloud(),
    initialized(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    goal_vertex_id(""),
    runs_since_path_calc(0),
    nav_goal_set_(false) {
    drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
    viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
    local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
    global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
    InitRosHeader("base_link", &drive_msg_.header);
    map_.Load(map_file);

    // Initialize a Map of string & vector of int using initializer_list
    //graph = { { "HiHoHee", { "hi", "ho", "hee" } }};
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    std::cout << "NEW NAV GOAL --- " << loc.x() << ", " << loc.y() << "\n";
    nav_goal_set_ = true;
    MakeGraph();
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    robot_loc_ = loc;
    robot_angle_ = angle;
    //std::cout << "UPDATED LOCATION --- " << loc.x() << ", " << loc.y() << "\n";
}

double Euclid2D(const double x, const double y) {
    return sqrt(Sq(x) + Sq(y));
}

void Navigation::MakeGraph() {
    graph.clear();

    float min_map_x = std::numeric_limits<float>::max();
    float max_map_x = -std::numeric_limits<float>::max();
    float min_map_y = std::numeric_limits<float>::max();
    float max_map_y = -std::numeric_limits<float>::max();

    // find the bounding x and y values for the graph
    for (geometry::line2f line : map_.lines) {
        if (line.p0.x() < min_map_x || line.p1.x() < min_map_x) {
            min_map_x = std::min(line.p0.x(), line.p1.x());
        } if (line.p0.x() > max_map_x || line.p1.x() > max_map_x) {
            max_map_x = std::max(line.p0.x(), line.p1.x());
        } if (line.p0.y() < max_map_x || line.p1.y() < max_map_x) {
            min_map_y = std::min(line.p0.y(), line.p1.y());
        } if (line.p0.y() > max_map_x || line.p1.y() > max_map_x) {
            max_map_y = std::max(line.p0.y(), line.p1.y());
        }
    }

    const float grid_space = 0.5;
    int height = abs(max_map_x - min_map_x) / grid_space;
    int width = abs(max_map_y - min_map_y) / grid_space;
    float min_goal_vert_dist = std::numeric_limits<float>::max();

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            Vertex* new_vertex = new Vertex;
            for (int i_ = i -1; i_ <= i + 1; i_++) {
                for (int j_ = j -1; j_ <= j + 1; j_++) {
                    if (!(i_ == i && j_ == j)
                            && i_ >= 0 && i_ < height
                            && j_ >= 0 && j_ < width) {
                        string neighbor_id = std::to_string(i_) + std::to_string(j_);
                        //std::cout << "id1 " << neighbor_id << "\n";
                        new_vertex->neighbors.push_back(neighbor_id);
                    }
                }
            }
            new_vertex->id = std::to_string(i) + std::to_string(j);
            //std::cout << "i, j --- " << i << ", " << j << "\n";
            //std::cout << "id2 " << new_id << " || " << "\n";
            //std::cout << "id2 " << new_vertex->id << " ** " << "\n";
            Vector2f new_vertex_loc(min_map_x + i * 0.5, min_map_y + j * 0.5);
            float goal_dist_diff = Euclid2D(new_vertex_loc.x() - nav_goal_loc_.x(),
                                   new_vertex_loc.y() - nav_goal_loc_.y());
            if (goal_dist_diff < min_goal_vert_dist) {
                min_goal_vert_dist = goal_dist_diff;
                goal_vertex_id = new_vertex->id;
            }
            new_vertex->loc = new_vertex_loc;
            graph.insert(std::pair<string, Vertex>(new_vertex->id, *new_vertex));
            delete new_vertex;
        }
    }

    std::cout << "graph size? " << graph.size()<< "\n";

    CalculatePath();
}

void Navigation::CalculatePath() {
    // find the closest vertex to the starting point
    string start_vertex_id = "";
    float min_start_vert_dist = std::numeric_limits<float>::max();
    for (const auto &v : graph) {
        float start_dist_diff = Euclid2D(v.second.loc.x() - robot_loc_.x(),
                                v.second.loc.y() - robot_loc_.y());
        if (start_dist_diff < min_start_vert_dist) {
            min_start_vert_dist = start_dist_diff;
            start_vertex_id = v.first;
        }
    }

    std::cout << "start loc  --- " << robot_loc_.x() << ", " << robot_loc_.y() << "\n";
    Vertex starrrt = graph[start_vertex_id];
    std::cout << "start vertex loc  --- " << starrrt.loc.x() << ", " << starrrt.loc.y() << "\n";
    std::cout << "goal loc  --- " << nav_goal_loc_.x() << ", " << nav_goal_loc_.y() << "\n";
    Vertex goalll = graph[goal_vertex_id];
    std::cout << "goal vertex loc  --- " << goalll.loc.x() << ", " << goalll.loc.y() << "\n";


    SimpleQueue<string, float> frontier;
    frontier.Push(start_vertex_id, 0);
    std::map<string, string> parent;
    parent.insert(std::pair<string, string>(start_vertex_id, ""));
    std::map<string, float> cost;
    cost.insert(std::pair<string, float>(start_vertex_id, 0));

    int tr = 0;
    // TODO check if instersect
    while (!frontier.Empty()) {

        tr++;
        string current_id = frontier.PopLow();
        Vertex current = graph[current_id];
        //std::cout << "neighbors???  " << current.neighbors.size() << "\n";
        if (current_id.compare(goal_vertex_id) == 0) {
            //std::cout << "num iters BREAK --- " << tr << " --- " << "\n";
            std::cout << "parent size BREAK: " << parent.size() << " --- " << "\n";
            break;
        }
        //std::cout << "neighbors???  " << current.neighbors.size() << "\n";
        for (string next_id : current.neighbors) {
            Vertex next = graph[next_id];
            float edge_weight = Euclid2D(next.loc.x() - current.loc.x(),
                                next.loc.y() - current.loc.y());
            float new_cost = cost[current_id] + edge_weight;
            if (cost.count(next_id) == 0 || new_cost < cost[next_id]) {
                std::cout << " ! new priority queue insert ! " << "\n";
                cost.insert(std::pair<string, float>(next_id, new_cost));
                frontier.Push(next_id, new_cost + Euclid2D(nav_goal_loc_.x() -
                              next.loc.x(), nav_goal_loc_.y() - next.loc.y()));
                parent.insert(std::pair<string, string>(next_id, current_id));
            }
        }
        std::cout << "--- END " << tr << " CURRENT --- " << "\n";
    }

    int tr2 = 0;
    while (!frontier.Empty()) {
        tr2++;
        string current_id = frontier.Pop();
    }
    //
    // std::cout << "parent size --- " << parent.size() << " --- " << "\n";
    //std::cout << "frontier size --- " << tr2 << " --- " << "\n";

    // visualize planned path
    for (const auto &v : parent) {
        std::string v1_id = v.first;
        Vertex v1 = graph[v1_id];
        Vector2f p1(v1.loc.x(), v1.loc.y());
        std::string v2_id = v.second;
        Vertex v2 = graph[v2_id];
        Vector2f p2(v2.loc.x(), v2.loc.y());
        visualization::DrawLine(p1, p2, 0xFFB8D3, local_viz_msg_);
    }
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    if (!initialized && loc.x() != 0) {
        initialized = true;
    }
    robot_loc_ = loc;
    robot_angle_ = angle;
    robot_vel_ = vel;
    robot_omega_ = ang_vel;
}



void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    visualization::ClearVisualizationMsg(local_viz_msg_);
    point_cloud.clear();
    for (Vector2f point : cloud) {
        point_cloud.push_back(point);
        //Vector2f global_point = GlobalizePoint(point);
        //visualization::DrawCross(global_point, .01, 0xFF0000, local_viz_msg_);
    }
    //viz_pub_.publish(local_viz_msg_);
}

// TODO: remove this when local visualization is fixed
Vector2f Navigation::GlobalizePoint(const Vector2f& local_point) {
    float range = Euclid2D(local_point.x(), local_point.y());
    float angle_orig = atan2(local_point.y(), local_point.x());
    float angle = robot_angle_ + angle_orig;
    Vector2f global_point(range * cos(angle), range * sin(angle));
    global_point += robot_loc_;
    return global_point;
}

double CalcVDelta(const double v_0, const double t, const double d) {
    double v_delta;
    // accelerate for 1 step
    double a_max = 3;
    double a_min = -3;
    double v_f = v_0 + a_max*t;
    double x_1 = t*(v_0 + v_f)/2;
    double t_2 = v_f/-a_min;
    double x_2 = t_2*(v_f/2);
    if (x_1 + x_2 <= d) {
        v_delta = a_max * t;
    } else {
        // cruise for 1 step
        x_1 = t*v_0;
        t_2 = v_0/-a_min;
        x_2 = t_2*(v_0/2);
        if (x_1 + x_2 <= d) {
            v_delta = 0;
        } else {
            // decelerate for 1 step
            double a = (v_0 * v_0)/(2*d);
            v_delta = -a * t;
        }
    }
    return v_delta;
}

// Uses local message
void Navigation::DrawCar(const Vector2f& local_point, uint32_t color, float angle) {
    // TODO draw car at angle
    Vector2f p1(local_point.x(), local_point.y() + w + angle);
    Vector2f p2(local_point.x() + h, local_point.y() + w);
    Vector2f p3(local_point.x() + h, local_point.y() - w);
    Vector2f p4(local_point.x(), local_point.y() - w);
    visualization::DrawLine(GlobalizePoint(p1), GlobalizePoint(p2), color, local_viz_msg_);
    visualization::DrawLine(GlobalizePoint(p2), GlobalizePoint(p3), color, local_viz_msg_);
    visualization::DrawLine(GlobalizePoint(p3), GlobalizePoint(p4), color, local_viz_msg_);
    visualization::DrawLine(GlobalizePoint(p4), GlobalizePoint(p1), color, local_viz_msg_);
}

void Navigation::Run() {
    if (!initialized)
        return;

    // constants
    float curv_inc = .2;
    float dist = 3.0;

    // relative goal
    Vector2f goal(dist, 0.0);

    // visuals
    visualization::ClearVisualizationMsg(local_viz_msg_);
    DrawCar(Vector2f(0,0), 0xFF0000, 0.0);
    visualization::DrawCross(GlobalizePoint(goal), .1, 0xFF0000, local_viz_msg_);

    if (nav_goal_set_) {
        visualization::DrawCross(nav_goal_loc_, .15, 0xFFB8D3, local_viz_msg_);
        if (runs_since_path_calc > 5) {
            CalculatePath();
            runs_since_path_calc = 0;
        } else {
            runs_since_path_calc++;
        }
    }

    // evaluate possible paths
    float best_curv = 0;
    float best_score = -9999.0;
    float best_fpl = 0;

    for (float curv = -1; curv <= 1; curv += curv_inc) {
        float fpl;
        float clearance = .2;
        float goal_dist;
        Vector2f dest;

        if (abs(curv) < .05) {
            curv = 0;
            // going straight
            fpl = 3;
            for (Vector2f point : point_cloud)
                if (abs(point.y()) <= w)
                    fpl = std::min(fpl, point.x() - h);
            for (Vector2f point : point_cloud)
                if (point.x() >= 0 && point.x() <= fpl + h)
                    // if the point is not behind the car and within the path
                    // calculate clearance
                    clearance = std::min(clearance, abs(point.y()) - w);
            goal_dist = Euclid2D(abs(fpl - goal.x()), goal.y());
            dest = Vector2f(fpl, 0);
        } else {
            float r = 1.0/curv;
            Vector2f g(goal.x(), goal.y());

            // std::vector<Eigen::Vector2f> local_points;
            // for (Vector2f point : point_cloud) {
            //     local_points.push_back(point);
            // }

            // if turning right, flip all points over x axis
            // if (r < 0) {
            //     r = -r;
            //     for (Vector2f point : local_points) {
            //         //std::cout << "point y BEFORE --- " << point.y() << " | ";
            //         //point.y() = -point.y();
            //         //std::cout << "point y AFTER  --- " << point.y() << "\n";
            //     }
            //     g.y() = -g.y();
            // }

            // Assumes goal is straight ahead dist meters
            fpl = abs(r) * atan2(dist, abs(r));

            if (r < 0) {
                // turning right
                double r_1 = abs(r + w);
                double r_2 = Euclid2D(r - w, h);
                double omega = atan2(h, abs(r + w));

                // compute free path length
                for (Vector2f point : point_cloud) {
                    if (point.x() < 0)
                        // point is behind car
                        continue;

                    //std::cout << "point y AFTER  --- " << point.y() << "\n";
                    double r_point = Euclid2D(point.x(), point.y() - r);
                    double theta = atan2(point.x(), abs(r - point.y()));

                    if (r_point >= r_1 && r_point <= r_2 && theta > 0) {
                        // the point is an obstable
                        //float curv_dist = r * (theta - omega) - 0.001;
                        float curv_dist = r * (theta - omega);
                        curv_dist = -curv_dist;
                        if (curv_dist < -0.05)
                            continue;
                        fpl = std::min(fpl, curv_dist);
                    }
                }

                // find clearance of curved path
                for (Vector2f point : point_cloud) {
                    if (point.x() < 0)
                        continue;
                    double r_point = Euclid2D(point.x(), point.y() - r);
                    double theta = atan2(point.x(), abs(r - point.y()));
                    float curv_dist = r * (theta - omega);
                    if (abs(curv_dist) <= fpl && curv_dist <= 0) {
                        float clear_curr = clearance;
                        if (r_point > abs(r)) {
                            clear_curr = r_point - r_2;
                        } else if (r_point < abs(r)) {
                            clear_curr = r_1 - r_point;
                        }

                        //float clear_curr = std::min(r_1 - r_point, r_point - r_2);
                        clearance = std::min(clearance, clear_curr);
                    }
                }
            } else {
                // turning left
                double r_1 = r - w;
                double r_2 = Euclid2D(r + w, h);
                double omega = atan2(h, r - w);

                // compute free path length
                for (Vector2f point : point_cloud) {
                    if (point.x() < 0)
                        // point is behind car
                        continue;

                    //std::cout << "point y AFTER  --- " << point.y() << "\n";
                    double r_point = Euclid2D(point.x(), point.y() - r);
                    double theta = atan2(point.x(), r - point.y());

                    if (r_point >= r_1 && r_point <= r_2 && theta > 0) {
                        // the point is an obstable
                        float curv_dist = r * (theta - omega);
                        if (curv_dist < -.05)
                            continue;
                        fpl = std::min(fpl, curv_dist);
                    }
                }

                // find clearance of curved path
                for (Vector2f point : point_cloud) {
                    if (point.x() < 0)
                        continue;
                    double r_point = Euclid2D(point.x(), point.y() - r);
                    double theta = atan2(point.x(), r - point.y());
                    float curv_dist = r * (theta - omega);
                    if (curv_dist <= fpl && curv_dist >= 0) {
                        float clear_curr = clearance;
                        if (r_point > r) {
                            clear_curr = r_point - r_2;
                        } else if (r_point < r) {
                            clear_curr = r_1 - r_point;
                        }

                        clearance = std::min(clearance, clear_curr);
                    }
                }
            }

            float rad = fpl / r;
            float dest_x = r * sin(rad);
            float dest_y = r - r * cos(rad);
            dest = Vector2f(dest_x, dest_y);
            goal_dist = Euclid2D(dest_x - goal.x(), dest_y - goal.y());
        }
        float w1 = .01;
        float w2 = -.2;
        float score = fpl + w1 * clearance + w2 * goal_dist;
        //std::cout << "curv " << curv << " fpl " << fpl << " clearance " << clearance << " goal_dist " << goal_dist << "\n";
        if (score > best_score) {
            best_score = score;
            best_curv = curv;
            best_fpl = fpl;
        }
        visualization::DrawPathOption(curv, fpl, clearance, local_viz_msg_);
        //DrawCar(dest, 0xFF00FF);
    }

    visualization::DrawPathOption(best_curv, best_fpl, 0, local_viz_msg_);
    viz_pub_.publish(local_viz_msg_);
    if (best_fpl <= 0.01)
        return;
    // 1d TOC
    double t = 1.0/20.0;
    double v_0 = Euclid2D(robot_vel_.x(), robot_vel_.y());
    double v_delta;

    float d = best_fpl;
    v_delta = CalcVDelta(v_0, t, d);
    // account for latency
    // TODO: have other calculations account for latency as well
    double latency = .05;
    d = best_fpl - (v_0 +(v_delta)/2) * latency;
    v_delta = CalcVDelta(v_0, t, d);

    double target_v = v_0 + v_delta;
    double max_v = 1;
    target_v = std::min(target_v, max_v);
    target_v = std::max(target_v, 0.0);

    drive_msg_.velocity = target_v;
    drive_msg_.curvature = best_curv;
    drive_pub_.publish(drive_msg_);
  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
}

}  // namespace navigation
