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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/math/statistics.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
// using std::sqrt;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 100, "Number of particles");

// Fill in the body of these functions and create helpers as needed
// in order to implement localization using a particle filter.

// Milestone 2 will be implemented here.

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    particles_(50),
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    mean_loc(0, 0),
    mean_angle(0),
    updates_since_resample(0) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
    const Vector2f kLaserLoc(0.2 * cos(angle), 0.2 * sin(angle));
    const Vector2f laser_loc = loc + kLaserLoc;
    float laser_angle = angle + angle_min;
    float laser_angle_incr = (angle_max - angle_min) / num_ranges;
    //const float range_diff = range_max - range_min;

    for (int i = 0; i < num_ranges; i++) {
        //TODO include range_min, range_diff
        float laser_x = laser_loc.x() + range_max * cos(laser_angle);
        float laser_y = laser_loc.y() + range_max * sin(laser_angle);
        bool collides = false;
        Vector2f laser_line(laser_x, laser_y);
        Vector2f intersection;
        float closest_intersection_dist = range_max;
        for (geometry::line2f line : map_.lines) {
            if (line.Intersection(laser_loc, laser_line, &intersection)) {
                collides = true;
                float intersection_dist = sqrt(Sq(intersection.x() - laser_loc.x()) + Sq(intersection.y() - laser_loc.y()));
                if (intersection_dist < closest_intersection_dist) {
                    closest_intersection_dist = intersection_dist;
                }
            }
        }

        if (collides) {
            laser_x = laser_loc.x() + closest_intersection_dist * cos(laser_angle);
            laser_y = laser_loc.y() + closest_intersection_dist * sin(laser_angle);
            laser_line = Vector2f(laser_x, laser_y);
        }

        scan_ptr->push_back(laser_line);
        laser_angle += laser_angle_incr;
    }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
      vector<float> predicted_ranges;
      // TODO: does this function wrap angles around for us
      map_.GetPredictedScan(p_ptr->loc, range_min, range_max, angle_min + p_ptr->angle,
                                  angle_max + p_ptr->angle, ranges.size(), &predicted_ranges);
      // compare predicted_ranges with ranges
      float particle_likelihood = 1.0;
      const float stddev = 0.05;
      const float gamma = 0.1;
      const float d_short = 0.2;
      const float d_long = 0.3;
      for (unsigned i = 0; i < ranges.size(); i += 10) {
          float single_ray_prob = 0;
          if (predicted_ranges[i] >= range_min && predicted_ranges[i] <= range_max) {
              if (predicted_ranges[i] < ranges[i] - d_short) {
                  single_ray_prob = Sq(d_short) / Sq(stddev);
              } else if (predicted_ranges[i] > ranges[i] + d_long) {
                  single_ray_prob = Sq(d_long) / Sq(stddev);
              } else {
                  single_ray_prob = Sq(ranges[i] - predicted_ranges[i]) / Sq(stddev);
              }
          }
          particle_likelihood += single_ray_prob;
      }
      p_ptr->weight = -1.0 * gamma * particle_likelihood;
}

void ParticleFilter::GetBestHypothesisScan(const Vector2f& loc,
                                            const float angle,
                                            const vector<float>& ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
    const Vector2f kLaserLoc(0.2 * cos(angle), 0.2 * sin(angle));
    const Vector2f laser_loc = loc + kLaserLoc;
    float laser_angle = angle + angle_min;
    float laser_angle_incr = (angle_max - angle_min) / ranges.size();

    for (uint i = 0; i < ranges.size(); i++) {
        float laser_x = laser_loc.x() + ranges[i] * cos(laser_angle);
        float laser_y = laser_loc.y() + ranges[i] * sin(laser_angle);
        Vector2f laser_line(laser_x, laser_y);
        scan_ptr->push_back(laser_line);
        laser_angle += laser_angle_incr;
    }
}

void ParticleFilter::Resample() {
    for (Particle& particle : particles_) {
        particle.weight -= best_guess_particle.weight;
        particle.weight = exp(particle.weight);
    }
    std::vector<Particle> resampled_particles;
    float W = 0.0;
    for (Particle& particle : particles_) {
        W += particle.weight;
    }
    for (unsigned int _ = 0; _ < particles_.size(); _++) {
        float w = 0;
        float x = rng_.UniformRandom(0, W);
        for (Particle& particle : particles_) {
            w += particle.weight;
            if (w > x) {
                Particle duplicate_particle(particle);
                resampled_particles.push_back(duplicate_particle);
                break;
            }
        }
    }
    //std::cout << "Orig:\n";
    //for (Particle& particle : particles_) {
    //    std::cout << particle.weight << "\n";
    //}
    //std::cout << "Resample\n";
    //for (Particle& particle : resampled_particles) {
    //    std::cout << particle.weight << "\n";
    //}
    //std::cout << "\n";
    // Is this mem leak?
    particles_ = resampled_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &particles_[0]);
    Particle highest_weight_particle = particles_[0];
    for (Particle& particle : particles_) {
        Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
        if (particle.weight > highest_weight_particle.weight) {
            highest_weight_particle = particle;
        }
    }
    best_guess_particle = highest_weight_particle;
    if (updates_since_resample > 3) {
        Resample();
        updates_since_resample = 0;
    } else {
        updates_since_resample++;
    }
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
    float k1 = .2;
    float k2 = .5;
    float k3 = .2;
    float k4 = .5;

    if (odom_initialized_) {
        float x_total = 0;
        float y_total = 0;
        float sin_theta_total = 0;
        float cos_theta_total = 0;
        Vector2f loc_delta = odom_loc - prev_odom_loc_;
        float r_delta = loc_delta.y() / sin(odom_angle);
        float delta_theta_hat = math_util::AngleDiff(odom_angle, prev_odom_angle_);

        for (Particle& particle : particles_) {
            float delta_x_hat = r_delta * cos(particle.angle);
            float delta_y_hat = r_delta * sin(particle.angle);
            float delta_x = rng_.Gaussian(delta_x_hat, k1 * sqrt(Sq(delta_x_hat)
                                + Sq(delta_y_hat)) + k2 * abs(delta_theta_hat));
            float delta_y = rng_.Gaussian(delta_y_hat, k1 * sqrt(Sq(delta_x_hat)
                                + Sq(delta_y_hat)) + k2 * abs(delta_theta_hat));
            float delta_theta = rng_.Gaussian(delta_theta_hat, k3 * sqrt(Sq(delta_x_hat)
                                + Sq(delta_y_hat)) + k4 * abs(delta_theta_hat));
            // Check for collision
            bool collides = false;
            Vector2f intersection;
            Vector2f p1 = particle.loc;
            Vector2f p2 = particle.loc + Vector2f(delta_x, delta_y);

            for (geometry::line2f line : map_.lines) {
                if (line.Intersection(p1, p2, &intersection)) {
                    collides = true;
                    break;
                }
            }

            if (collides) {
                particle.loc = intersection;
            } else {
                particle.loc = p2;
            }

            particle.angle += delta_theta;
            particle.weight = 1;
            x_total += particle.loc.x();
            y_total += particle.loc.y();
            sin_theta_total += sin(particle.angle);
            cos_theta_total += cos(particle.angle);
        }
        mean_loc = Vector2f(x_total / particles_.size(), y_total / particles_.size());
        mean_angle = atan2(sin_theta_total / particles_.size(), cos_theta_total / particles_.size());
    } else {
        odom_initialized_ = true;
    }
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
    float k = .1;
    float k2 = .1;
    for (Particle& particle : particles_) {
        float x = rng_.Gaussian(loc.x(), k);
        float y = rng_.Gaussian(loc.y(), k);
        particle.loc = Vector2f(x, y);
        particle.angle = rng_.Gaussian(angle, k2);
        particle.weight = 1;
    }
    odom_initialized_ = false;
    map_.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {
    *loc = best_guess_particle.loc;
    *angle = best_guess_particle.angle;
}

}  // namespace particle_filter
