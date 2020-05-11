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
#include "shared/math/line2d.h"

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
    runs_since_path_calc(6),
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
}

double Euclid2D(const double x, const double y) {
    return sqrt(Sq(x) + Sq(y));
}

// Note: What if a wall was completely inside the safety circle?
bool Navigation::TooCloseToWall(const Eigen::Vector2f vertex_loc) {
    Vector2f intersection(0.0, 0.0);
    float sd = 0;
    for (geometry::line2f line : map_.lines) {
        if (geometry::FurthestFreePointCircle(line.p0, line.p1, vertex_loc, w, &sd, &intersection)) {
            return true;
        }
    }
    return false;
}

string Navigation::GetClosestVertexID(const Vector2f point) {
    string closest_vertex_id = "";
    float min_dist = std::numeric_limits<float>::max();
    for (const auto &v : graph) {
        float dist = Euclid2D(v.second.loc.x() - point.x(),
                              v.second.loc.y() - point.y());
        if (dist < min_dist) {
            min_dist = dist;
            closest_vertex_id = v.first;
        }
    }
    return closest_vertex_id;
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    if (!nav_goal_set_) {
        nav_goal_set_ = true;
        MakeGraph();
    }
    goal_vertex_id = GetClosestVertexID(nav_goal_loc_);
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    robot_loc_ = loc;
    robot_angle_ = angle;
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
        } if (line.p0.y() < min_map_y || line.p1.y() < min_map_y) {
            min_map_y = std::min(line.p0.y(), line.p1.y());
        } if (line.p0.y() > max_map_y || line.p1.y() > max_map_y) {
            max_map_y = std::max(line.p0.y(), line.p1.y());
        }
    }

    const float grid_space = 0.4;
    int height = abs(max_map_x - min_map_x) / grid_space;
    int width = abs(max_map_y - min_map_y) / grid_space;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            Vertex* new_vertex = new Vertex;
            //new_vertex->id = std::to_string(i) + "," + std::to_string(j);
            Vector2f new_vertex_loc(min_map_x + i * grid_space, min_map_y + j * grid_space);

            if (!TooCloseToWall(new_vertex_loc)) {
                new_vertex->id = std::to_string(new_vertex_loc.x()) + "," + std::to_string(new_vertex_loc.y());
                for (int i_ = i -1; i_ <= i + 1; i_++) {
                    for (int j_ = j -1; j_ <= j + 1; j_++) {
                        if (!(i_ == i && j_ == j)
                                && i_ >= 0 && i_ < height
                                && j_ >= 0 && j_ < width) {
                            Vector2f neighbor_vertex_loc(min_map_x + i_ * grid_space, min_map_y + j_ * grid_space);
                            // make sure edge won't collide with map or isn't too close
                            bool collides = false;
                            string neighbor_id = std::to_string(neighbor_vertex_loc.x()) + "," + std::to_string(neighbor_vertex_loc.y());
                            if (graph.count(neighbor_id) > 0) {
                                // first check if this vertex is already a neighbor
                                std::vector<std::string> n = graph[neighbor_id].neighbors;
                                collides = std::find(n.begin(), n.end(), new_vertex->id) == n.end();
                            } else {
                                for (geometry::line2f line : map_.lines) {
                                    if (line.Intersects(new_vertex_loc, neighbor_vertex_loc)) {
                                        collides = true;
                                        break;
                                    }
                                }
                            }
                            if (!collides) {
                                //string neighbor_id = std::to_string(i_) + "," + std::to_string(j_);
                                //std::cout << "id1 " << neighbor_id << "\n";
                                new_vertex->neighbors.push_back(neighbor_id);
                            }
                        }
                    }
                }
                //std::cout << "i, j --- " << i << ", " << j << "\n";
                //std::cout << "id2 " << new_id << " || " << "\n";
                //std::cout << "id2 " << new_vertex->id << " ** " << "\n";
                new_vertex->loc = new_vertex_loc;
                graph.insert(std::pair<string, Vertex>(new_vertex->id, *new_vertex));
            }
            delete new_vertex;
        }
    }
}


// Vertex JumpPoint(Vertex current, float x_diff, float y_diff) {
//     float next_x = current.x() + x_diff;
//     float next_y = current.y() + y_diff;
//     string next_id = std::to_string(next_x) + "," + std::to_string(next_y);
//     Vertex goal_vertex = graph[goal_vertex_id];
//
//     if (graph.count(next_id) == 0 || next_CollidesWithMap(current.loc, nextloc)) {
//         return null;
//     }
//
//     if (next_x == goal_vertex.x() && next_y == goal_vertex.y()) {
//         return goal_vertex;
//     }
//
//     Vertex next_vertex = graph[next_id];
//     const int max_neighbors = 8;
//
//     // going diagonal
//     if (x_diff != 0 && y_diff != 0) {
//         if (true) {
//             // is forced neighbor
//             return next_vertex;
//         }
//
//         // check horizontal and vertical directions for forced neighbors
//         if (jump(next_x, next_y, x_diff, 0) != null ||
//             jump(next_x, next_y, 0, y_diff) != null)
//         {
//             return next_vertex;
//         }
//     } else if (x_diff != 0) {
//         // horizontal jumping
//         if (true) {
//             // is a forced neighbor
//             return next_vertex;
//         }
//     } else {
//         // vertical jumping
//         if (true) {
//             // is a forced neighbor
//             return next_vertex;
//         }
//     }
//
//     return JumpPoint(next_vertex, x_diff, y_diff);
// }

void Navigation::CalculatePath() {
    string start_vertex_id = GetClosestVertexID(robot_loc_);
    SimpleQueue<string, float> frontier;
    frontier.Push(start_vertex_id, 0);
    std::map<string, string> parent;
    parent.insert(std::pair<string, string>(start_vertex_id, ""));
    std::map<string, float> cost;
    cost.insert(std::pair<string, float>(start_vertex_id, 0));

    std::map<string, string> nav_path;

    while (!frontier.Empty()) {
        string current_id = frontier.Pop();
        Vertex current = graph[current_id];

        if (current_id.compare(goal_vertex_id) == 0) {
            break;
        }

        for (string next_id : current.neighbors) {
            if (graph.count(next_id) > 0) {
                Vertex next = graph[next_id];
                float edge_weight = Euclid2D(next.loc.x() - current.loc.x(),
                                    next.loc.y() - current.loc.y());
                float new_cost = cost[current_id] + edge_weight;
                if (cost.count(next_id) == 0 || new_cost < cost[next_id]) {
                    //Vertex jp = JumpPoint(current, next.x() - current.x(), next.y() - current.y());
                    cost[next_id] = new_cost;
                    float heuristic = Euclid2D(nav_goal_loc_.x() - next.loc.x(),
                            nav_goal_loc_.y() - next.loc.y());
                    float inflation = 1.2;
                    frontier.Push(next_id, -1 * (new_cost + inflation * heuristic));
                    parent[next_id] = current_id;
                }
            }
        }
    }

    // extract planned path
    planned_path.clear();
    planned_path.push_back(graph[goal_vertex_id].loc);
    for (string v = goal_vertex_id; v.compare(start_vertex_id) != 0; v = parent[v]) {
        if (parent[v].compare("") == 0) {
            // this  means no path was found, which shouldn't happen
            std::cout << "Parent was empty!\n";
            break;
        }
        planned_path.push_back(graph[parent[v]].loc);
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
    }
}

Vector2f Navigation::GlobalizePoint(const Vector2f& local_point) {
    float range = Euclid2D(local_point.x(), local_point.y());
    float angle_orig = atan2(local_point.y(), local_point.x());
    float angle = robot_angle_ + angle_orig;
    Vector2f global_point(range * cos(angle), range * sin(angle));
    global_point += robot_loc_;
    return global_point;
}

Vector2f Navigation::LocalizePoint(const Vector2f& global_point) {
    Vector2f local_point(global_point - robot_loc_);
    float range = Euclid2D(local_point.x(), local_point.y());
    float angle_orig = atan2(local_point.y(), local_point.x());
    float angle = angle_orig - robot_angle_;
    local_point = Vector2f(range * cos(angle), range * sin(angle));
    return local_point;
}

double CalcVDelta(const double v_0, const double t, const double d) {
    double v_delta;
    // accelerate for 1 step
    double a_max = 3;
    double a_min = -3;
    double v_f = v_0 + a_max * t;
    double x_1 = t * (v_0 + v_f) / 2;
    double t_2 = v_f / -a_min;
    double x_2 = t_2 * (v_f / 2);
    if (x_1 + x_2 <= d) {
        v_delta = a_max * t;
    } else {
        // cruise for 1 step
        x_1 = t * v_0;
        t_2 = v_0 / -a_min;
        x_2 = t_2 * (v_0 / 2);
        if (x_1 + x_2 <= d) {
            v_delta = 0;
        } else {
            // decelerate for 1 step
            double a = (v_0 * v_0) / (2 * d);
            v_delta = -a * t;
        }
    }
    return v_delta;
}

// Uses local message
void Navigation::DrawCar(const Vector2f& local_point, uint32_t color, float angle) {
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
    const float curv_inc = 0.2;
    float carrot_dist = 1.2;

    // relative goal
    Vector2f carrot(carrot_dist, 0.0);

    // visuals
    visualization::ClearVisualizationMsg(local_viz_msg_);
    DrawCar(Vector2f(0,0), 0xFF0000, 0.0);

    if (nav_goal_set_) {
        // don't move if reached goal
        if (Euclid2D(robot_loc_.x() - nav_goal_loc_.x(),
            robot_loc_.y() - nav_goal_loc_.y()) < 0.5)
        {
            return;
        }

        if (runs_since_path_calc > 3) {
            CalculatePath();
            runs_since_path_calc = 0;
        } else {
            runs_since_path_calc++;
        }
        bool found = false;
        for (unsigned int i = 0; i < planned_path.size() - 1; i++) {
            Vector2f p1 = planned_path[i];
            Vector2f p2 = planned_path[i + 1];
            float sd = 0;
            if (!found) {
                found = geometry::FurthestFreePointCircle(p1, p2, robot_loc_, carrot_dist, &sd, &carrot);
                carrot = LocalizePoint(carrot);
            }
            // draw planned path
            visualization::DrawLine(p1, p2, 0x11FF11, local_viz_msg_);
        }
        visualization::DrawCross(nav_goal_loc_, .15, 0x8B7FFF, local_viz_msg_);
    }
    visualization::DrawCross(GlobalizePoint(carrot), .1, 0xFF0000, local_viz_msg_);

    // evaluate possible paths
    float best_curv = 0;
    float best_score = -std::numeric_limits<float>::max();
    float best_fpl = 0;

    for (float curv = -1; curv <= 1; curv += curv_inc) {
        float fpl = carrot_dist;
        float clearance = .2;
        //float carrot_dist;
        Vector2f dest;

        if (abs(curv) < .05) {
            curv = 0;
            // going straight
            for (Vector2f point : point_cloud)
                if (abs(point.y()) <= w)
                    fpl = std::min(fpl, point.x() - h);
            for (Vector2f point : point_cloud)
                if (point.x() >= 0 && point.x() <= fpl + h)
                    // if the point is not behind the car and within the path
                    // calculate clearance
                    clearance = std::min(clearance, abs(point.y()) - w);
            carrot_dist = Euclid2D(abs(fpl - carrot.x()), carrot.y());
            dest = Vector2f(fpl, 0);
        } else {
            float r = 1.0 / curv;
            Vector2f g(carrot.x(), carrot.y());
            bool turn_right = false;

            if (r < 0) {
                r = -1 * r;
                turn_right = true;
            }

            // Assumes goal is straight ahead dist meters
            fpl = r * atan2(carrot_dist, r);
            double r_1 = r - w;
            double r_2 = Euclid2D(r + w, h);
            double omega = atan2(h, r - w);

            // compute free path length
            for (Vector2f point : point_cloud) {
                if (point.x() < 0)
                    // point is behind car
                    continue;
                float point_y = point.y();
                if (turn_right) {
                    point_y = -1 * point.y();
                }
                //std::cout << "point y AFTER  --- " << point.y() << "\n";
                double r_point = Euclid2D(point.x(), point_y - r);
                double theta = atan2(point.x(), r - point_y);
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
                float point_y = point.y();
                if (turn_right) {
                    point_y = -1 * point.y();
                }
                double r_point = Euclid2D(point.x(), point_y - r);
                double theta = atan2(point.x(), r - point_y);
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

            if (turn_right) {
                r = -1 * r;
            }

            float rad = fpl / r;
            float dest_x = r * sin(rad);
            float dest_y = r - r * cos(rad);
            dest = Vector2f(dest_x, dest_y);
            carrot_dist = Euclid2D(dest_x - carrot.x(), dest_y - carrot.y());
        }

        float w1 = 0.1;
        float w2 = -5;
        // make fpl weighted negatively if very small to avoid bad paths
        if (fpl < h && fpl > 0) {
            float bad_fpl_multiplier = -100;
            fpl = bad_fpl_multiplier * fpl;
        }
        float score = fpl + w1 * clearance + w2 * carrot_dist;
        std::cout << "curv " << curv << " fpl " << fpl << " clearance " << clearance << " carrot_dist " << carrot_dist << "\n";
        if (score > best_score) {
            best_score = score;
            best_curv = curv;
            best_fpl = fpl;
        }
        visualization::DrawPathOption(curv, fpl, clearance, local_viz_msg_);
    }

    visualization::DrawPathOption(best_curv, best_fpl, 0, local_viz_msg_);
    viz_pub_.publish(local_viz_msg_);
    if (best_fpl <= 0.01)
        return;
    // 1D TOC
    double t = 1.0/20.0;
    double v_0 = Euclid2D(robot_vel_.x(), robot_vel_.y());
    double v_delta;

    float d = best_fpl;
    v_delta = CalcVDelta(v_0, t, d);
    // account for latency
    double latency = 0.05;
    d = best_fpl - (v_0 + (v_delta) / 2) * latency;
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
