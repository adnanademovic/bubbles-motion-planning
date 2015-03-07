//
// Copyright (c) 2014, Adnan Ademovic
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "pqp-environment.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace {

inline double Dot(double x1, double y1, double z1,
    double x2, double y2, double z2) {
  return (x1 * x2 + y1 * y2 + z1 * z2);
}

inline double NormSquared(double x, double y, double z) {
  return Dot(x, y, z, x, y, z);
}

double PointDistanceToAxis(double pos_x, double pos_y, double pos_z,
                           double x, double y, double z) {
  double l = Dot(pos_x, pos_y, pos_z, x, y, z);
  return sqrt(NormSquared(pos_x - l * x, pos_y - l * y, pos_z - l * z));
}

double PointDistanceToVector(double pos_x, double pos_y, double pos_z,
                             double x, double y, double z) {
  double point_a_distance(NormSquared(pos_x, pos_y, pos_z));
  double point_b_distance(NormSquared(pos_x - x, pos_y - y, pos_z - z));
  double ux = -x;
  double uy = -y;
  double uz = -z;
  double normalizer = sqrt(NormSquared(ux, uy, uz));
  if (normalizer != 0.0 && normalizer != -0.0) {
    ux /= normalizer;
    uy /= normalizer;
    uz /= normalizer;
  }
  if (point_a_distance < point_b_distance) {
    double dx = pos_x;
    double dy = pos_y;
    double dz = pos_z;
    normalizer = sqrt(NormSquared(dx, dy, dz));
    if (normalizer != 0.0 && normalizer != -0.0) {
      dx /= normalizer;
      dy /= normalizer;
      dz /= normalizer;
    }
    normalizer = Dot(dx, dy, dz, -ux, -uy, -uz);
    return (normalizer > 0.0)
        ? sqrt(point_a_distance * (1.0 - normalizer * normalizer))
        : sqrt(point_a_distance);
  } else {
    double dx = pos_x - x;
    double dy = pos_y - y;
    double dz = pos_z - z;
    normalizer = sqrt(NormSquared(dx, dy, dz));
    if (normalizer != 0.0 && normalizer != -0.0) {
      dx /= normalizer;
      dy /= normalizer;
      dz /= normalizer;
    }
    normalizer = Dot(dx, dy, dz, ux, uy, uz);
    return (normalizer > 0.0)
        ? sqrt(point_b_distance * (1.0 - normalizer * normalizer))
        : sqrt(point_b_distance);
  }
}

// TODO: Make parsing more sophisticated and do other parsing besides bin STL.
// TODO: Fix parsing for big endian machines.
PQP_Model* ParseModel(const std::string& filename, int* counter,
                      transforms::Transformation* t,
                      double x, double y, double z, double* l, double* r) {
  FILE* f = fopen(filename.c_str(), "rb");
  uint8_t header[80];
  uint32_t triangle_count;
  fread(header, sizeof(header[0]), 80, f);
  fread(&triangle_count, sizeof(triangle_count), 1, f);
  PQP_REAL p[3][3];
  float normal[3];
  float input[3][3];
  uint16_t attribute;
  PQP_Model* model = new PQP_Model;
  model->BeginModel();
  *l = 0.0;
  *r = 0.0;
  for (uint32_t triangle = 0; triangle < triangle_count; ++triangle) {
    fread(normal, sizeof(normal[0]), 3, f);
    fread(input[0], sizeof(input[0][0]), 3, f);
    fread(input[1], sizeof(input[1][0]), 3, f);
    fread(input[2], sizeof(input[2][0]), 3, f);
    fread(&attribute, sizeof(attribute), 1, f);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        p[i][j] = static_cast<PQP_REAL>(input[i][j]);
    for (int tri = 0; tri < 3; ++tri) {
      t->MovePoint(p[tri]);
      *l = std::max(*l, Dot(p[tri][0], p[tri][1], p[tri][2], x, y, z));
      *r = std::max(*r, PointDistanceToVector(
            p[tri][0], p[tri][1], p[tri][2], x * (*l), y * (*l), z * (*l)));
    }
    model->AddTri(p[0], p[1], p[2], (*counter)++);
  }
  model->EndModel();
  fclose(f);
  return model;
}

void setPointPosition(
    double R[3][3], double T[3], double p[3], const std::vector<double>& t) {
  for (int i = 0; i < 3; ++i) {
    p[i] = T[i];
    for (int j = 0; j < 3; ++j)
      p[i] += R[i][j] * t[j];
  }
}

}  // namespace

// TODO: Use one config file to fetch all the data.
PqpEnvironment::PqpEnvironment(const std::string& configuration) {
  boost::property_tree::ptree config_tree;
  boost::property_tree::read_json(configuration, config_tree);

  std::vector<std::vector<double> > config;
  for (auto& item : config_tree.get_child("dh")) {
    config.emplace_back();
    for (auto& param : item.second)
      config.back().push_back(param.second.get_value<double>());
  }

  std::vector<int> parts_per_joint;
  for (auto& item : config_tree.get_child("parts_per_joint"))
    parts_per_joint.push_back(item.second.get_value<int>());
  std::vector<std::string> parts;
  for (auto& item : config_tree.get_child("parts"))
    parts.push_back(item.second.get_value<std::string>());
  std::string environment = config_tree.get<std::string>("environment");
  double max_underestimate = config_tree.get<double>("max_underestimate");

  part_count_ = parts.size();
  variance_ = max_underestimate / 2.0;
  is_joint_start_.resize(parts.size(), false);

  LoadDh(config);

  int part_index = 0;
  int segment_index = -1;
  int counter = 0;
  is_joint_start_[0] = true;
  for (int parts_c : parts_per_joint) {
    part_index += parts_c;
    if (part_index < part_count_)
      is_joint_start_[part_index] = true;
  }
  part_index = 0;
  for (const std::string& part : parts) {
    if (is_joint_start_[part_index])
      ++segment_index;
    double l, r;
    double x = dh_parameters_[segment_index]->coefficient(0, 3);
    double y = dh_parameters_[segment_index]->coefficient(1, 3);
    double z = dh_parameters_[segment_index]->coefficient(2, 3);
    double vector_length = sqrt(NormSquared(x, y, z));
    if (vector_length > 0.0) {
      x /= vector_length;
      y /= vector_length;
      z /= vector_length;
    }
    parts_.emplace_back(ParseModel(
          part, &counter, dh_inverted_[segment_index].get(), x, y, z, &l, &r));
    cylinders_.emplace_back(std::vector<double>{x * l, y * l, z * l}, r);
    part_index++;
  }
  double l, r;
  environment_.reset(ParseModel(
        environment, &counter, dh_inverted_[0].get(), 0.0, 0.0, 0.0, &l, &r));
}

bool PqpEnvironment::IsCollision(const std::vector<double>& q) const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  PQP_REAL R[3][3], T[3];
  PQP_REAL obs_R[3][3], obs_T[3];
  for (int i = 0; i < 3; ++i) {
    obs_T[i] = T[i] = 0.0;
    for (int j = 0; j < 3; ++j)
      obs_R[i][j] = R[i][j] = ((i==j) ? 1.0 : 0.0);
  }
  int segment = -1;
  for (int part = 0; part < part_count_; ++part) {
    if (is_joint_start_[part]) {
      if (segment > -1)
        dh_parameters_[segment]->ApplyTo(R, T);
      ++segment;
      transforms::Transformation rotation;
      rotation.Rotate(transforms::Transformation::Z, q[segment]);
      rotation.ApplyTo(R, T);
    }
    PQP_CollideResult collide_res;
    PQP_Collide(&collide_res, R, T, parts_[part].get(),
                obs_R, obs_T, environment_.get());

    if (collide_res.Colliding() != 0)
      return true;
  }
  return false;
}

PqpEnvironment::DistanceProfile PqpEnvironment::GetDistanceProfile(
    const std::vector<double>& q) const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  PQP_REAL R[3][3], T[3]; // used to identify joint positions
  PQP_REAL obs_R[3][3], obs_T[3]; // the fixed obstacle position
  std::vector<std::vector<double> > ax_P; // stores all joint positions
  std::vector<std::vector<double> > ax_O; // stores all axis orientations
  for (int i = 0; i < 3; ++i) {
    obs_T[i] = T[i] = 0.0;
    for (int j = 0; j < 3; ++j)
      obs_R[i][j] = R[i][j] = ((i==j) ? 1.0 : 0.0);
  }
  DistanceProfile distances;
  int segment = -1;

  double p[3], p_prev[3];
  for (int part = 0; part < part_count_; ++part) {
    if (is_joint_start_[part]) {
      if (segment > -1)
        dh_parameters_[segment]->ApplyTo(R, T);
      ++segment;
      ax_P.push_back({T[0], T[1], T[2]});
      ax_O.push_back({R[0][2], R[1][2], R[2][2]});
      transforms::Transformation rotation;
      rotation.Rotate(transforms::Transformation::Z, q[segment]);
      rotation.ApplyTo(R, T);
    }

    PQP_DistanceResult distance_res;
    PQP_Distance(&distance_res, R, T, parts_[part].get(), obs_R,
                 obs_T, environment_.get(), 10000000.0, variance_);
    distances.emplace_back(std::max(distance_res.Distance() - variance_, 0.0),
                           std::vector<double>(0));

    std::vector<double>* radiuses = &(distances.back().second);
    setPointPosition(R, T, p, cylinders_[part].first);
    for (int seg = 0; seg <= segment; ++seg) {
      double d_now = PointDistanceToAxis(
              p[0] - ax_P[seg][0], p[1] - ax_P[seg][1], p[2] - ax_P[seg][2],
              ax_O[seg][0], ax_O[seg][1], ax_O[seg][2]);
      double d_prev = PointDistanceToAxis(
              p_prev[0] - ax_P[seg][0], p_prev[1] - ax_P[seg][1],
              p_prev[2] - ax_P[seg][2],
              ax_O[seg][0], ax_O[seg][1], ax_O[seg][2]);
      radiuses->push_back(cylinders_[part].second + std::max(d_now, d_prev));
    }
    for (int i = 0; i < 3; ++i)
      p_prev[i] = p[i];
  }

  return distances;
}

int PqpEnvironment::part_count() const {
  return part_count_;
}

void PqpEnvironment::LoadDh(
    const std::vector<std::vector<double> >& configuration) {
  // each part gets transformed ignoring the last coordinate transform
  dh_inverted_.emplace_back(new transforms::Transformation);
  for (const std::vector<double>& param : configuration) {
    dh_parameters_.emplace_back(new transforms::Transformation);
    transforms::Transformation* t = dh_parameters_.back().get();
    t->Rotate(transforms::Transformation::Z, param[1]);
    t->Translate(param[0], 0.0, param[2]);
    t->Rotate(transforms::Transformation::X, param[3]);
    dh_inverted_.emplace_back(new transforms::Transformation);
    t = dh_inverted_.back().get();
    t->Rotate(transforms::Transformation::X, -param[3]);
    t->Translate(-param[0], 0.0, -param[2]);
    t->Rotate(transforms::Transformation::Z, -param[1]);
    t->Transform(*dh_inverted_[dh_inverted_.size() - 2]);
  }
}

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
