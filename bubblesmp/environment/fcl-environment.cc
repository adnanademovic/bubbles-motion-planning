//
// Copyright (c) 2015, Adnan Ademovic
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

#include "bubblesmp/environment/fcl-environment.h"

#include <algorithm>
#include <cmath>
#include <fstream>

#include <fcl/BVH/BVH_model.h>
#include <fcl/distance.h>

#include <glog/logging.h>
#include <google/protobuf/text_format.h>

#include "bubblesmp/environment/environment.pb.h"

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace {

constexpr double deg_to_rad() {
  return atan(1) / 45.0;
}

double PointDistanceToAxis(const fcl::Vec3f& point, const fcl::Vec3f& axis) {
  double l = point.dot(axis);
  return (point - axis * l).length();
}

double PointDistanceToVector(const fcl::Vec3f& point, const fcl::Vec3f& vect) {
  double point_a_distance(point.sqrLength());
  fcl::Vec3f d = point - vect;
  double point_b_distance(d.sqrLength());
  fcl::Vec3f u = -vect;
  u.normalize();
  if (point_a_distance < point_b_distance) {
    d = point;
    d.normalize();
    double c = d.dot(fcl::Vec3f(vect).normalize());
    return (c > 0.0)
        ? sqrt(point_a_distance * (1.0 - c * c))
        : sqrt(point_a_distance);
  } else {
    d.normalize();
    double c = d.dot((-vect).normalize());
    return (c > 0.0)
        ? sqrt(point_b_distance * (1.0 - c * c))
        : sqrt(point_b_distance);
  }
}

// TODO: Make parsing more sophisticated and do other parsing besides bin STL.
// TODO: Fix parsing for big endian machines.
fcl::BVHModel<fcl::OBBRSS>* ParseModel(
    const std::string& filename, fcl::Transform3f* t,
    double x, double y, double z, double* l, double* r) {
  FILE* f = fopen(filename.c_str(), "rb");
  uint8_t header[80];
  uint32_t triangle_count;
  fread(header, sizeof(header[0]), 80, f);
  fread(&triangle_count, sizeof(triangle_count), 1, f);
  float input[3][3];
  fcl::Vec3f p[3];
  float normal[3];
  uint16_t attribute;
  fcl::BVHModel<fcl::OBBRSS>* model = new fcl::BVHModel<fcl::OBBRSS>;
  model->beginModel();
  *l = 0.0;
  *r = 0.0;
  for (uint32_t triangle = 0; triangle < triangle_count; ++triangle) {
    fread(normal, sizeof(normal[0]), 3, f);
    fread(input[0], sizeof(input[0][0]), 3, f);
    fread(input[1], sizeof(input[1][0]), 3, f);
    fread(input[2], sizeof(input[2][0]), 3, f);
    fread(&attribute, sizeof(attribute), 1, f);
    for (int tri = 0; tri < 3; ++tri) {
      p[tri] = t->transform(fcl::Vec3f(
          input[tri][0], input[tri][1], input[tri][2]));
      *l = std::max(*l, p[tri].dot({x, y, z}));
      *r = std::max(*r, PointDistanceToVector(
          p[tri], {x * (*l), y * (*l), z * (*l)}));
    }
    model->addTriangle(p[0], p[1], p[2]);
  }
  model->endModel();
  fclose(f);
  return model;
}

}  // namespace

FclEnvironment::~FclEnvironment() {}

FclEnvironment::FclEnvironment(const std::string& configuration) {
  boost::filesystem::path config_file_path(configuration);
  config_file_path.remove_filename();
  std::ifstream fin(configuration);
  EnvironmentConfig config_pb;
  std::string input_string((std::istreambuf_iterator<char>(fin)),
                           std::istreambuf_iterator<char>());
  bool success = google::protobuf::TextFormat::ParseFromString(
      input_string, &config_pb);
  fin.close();
  CHECK(success) << "Failed parsing file: " << configuration;
  ConfigureFromPB(config_pb, config_file_path);
}

FclEnvironment::FclEnvironment(
    const EnvironmentConfig& configuration,
    const boost::filesystem::path& config_file_path) {
  ConfigureFromPB(configuration, config_file_path);
}

bool FclEnvironment::IsCollision(const std::vector<double>& q) const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  fcl::Transform3f pose;
  fcl::Matrix3f joint_angle_rotation;
  int segment = -1;
  for (int part = 0; part < part_count_; ++part) {
    if (is_joint_start_[part]) {
      if (segment > -1)
        pose *= *dh_parameters_[segment];
      ++segment;
      joint_angle_rotation.setEulerZYX(0.0, 0.0, q[segment] * deg_to_rad());
      pose *= fcl::Transform3f(joint_angle_rotation);
    }
    parts_[part]->setTransform(pose);

    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    fcl::collide(parts_[part].get(), environment_.get(), request, result);
    if (result.isCollision())
      return true;
  }
  return false;
}

EnvironmentInterface::DistanceProfile FclEnvironment::GetDistanceProfile(
    const std::vector<double>& q) const {
  std::lock_guard<std::mutex> guard(guard_mutex_);
  fcl::Transform3f pose;
  fcl::Matrix3f joint_angle_rotation;
  std::vector<fcl::Vec3f> ax_P; // stores all joint positions
  std::vector<fcl::Vec3f> ax_O; // stores all axis orientations
  EnvironmentInterface::DistanceProfile distances;
  int segment = -1;

  fcl::Vec3f p, p_prev;
  fcl::Matrix3f R;
  for (int part = 0; part < part_count_; ++part) {
    if (is_joint_start_[part]) {
      if (segment > -1)
        pose *= *dh_parameters_[segment];
      ++segment;
      ax_P.push_back(pose.getTranslation());
      R = pose.getRotation();
      ax_O.push_back({R(0, 2), R(1, 2), R(2, 2)});
      joint_angle_rotation.setEulerZYX(0.0, 0.0, q[segment] * deg_to_rad());
      pose *= fcl::Transform3f(joint_angle_rotation);
    }
    parts_[part]->setTransform(pose);

    fcl::DistanceRequest request(false, 1.0, variance_);
    fcl::DistanceResult result;
    fcl::distance(parts_[part].get(), environment_.get(), request, result);

    distances.emplace_back(
        std::max(result.min_distance, 0.0), std::vector<double>(0));

    std::vector<double>* radiuses = &(distances.back().second);
    p = pose.transform(cylinders_[part].first);
    for (int seg = 0; seg <= segment; ++seg) {
      radiuses->push_back(cylinders_[part].second + std::max(
          PointDistanceToAxis(p - ax_P[seg], ax_O[seg]),
          PointDistanceToAxis(p_prev - ax_P[seg], ax_O[seg])));
    }
    p_prev = p;
  }

  return distances;
}

std::vector<std::pair<double, double> > FclEnvironment::GetAngleRanges() const {
  return limits_;
}

void FclEnvironment::LoadDhAndRanges(const Robot& robot) {
  dh_inverted_.emplace_back(new fcl::Transform3f);
  for (const Segment& param : robot.segments()) {
    dh_parameters_.emplace_back(new fcl::Transform3f);
    fcl::Transform3f* t = dh_parameters_.back().get();

    fcl::Matrix3f rotations;
    rotations.setEulerZYX(0.0, 0.0, param.theta() * deg_to_rad());
    *t *= fcl::Transform3f(rotations);
    *t *= fcl::Transform3f(fcl::Vec3f(param.a(), 0.0, param.d()));
    rotations.setEulerZYX(param.alpha() * deg_to_rad(), 0.0, 0.0);
    *t *= fcl::Transform3f(rotations);

    dh_inverted_.emplace_back(new fcl::Transform3f(*t));
    fcl::Transform3f* t_inv = dh_inverted_.back().get();
    t_inv->inverse();
    *t_inv *= *dh_inverted_[dh_inverted_.size() - 2];
    CHECK (param.has_range() && param.range().has_min() &&
        param.range().has_max())
        << "Range missing from Protocol Buffer at:" << std::endl
        << param.DebugString();
    limits_.emplace_back(param.range().min(), param.range().max());
  }
}

void FclEnvironment::ConfigureFromPB(
    const EnvironmentConfig& configuration,
    const boost::filesystem::path& config_file_path) {
  CHECK(configuration.has_robot_config() || configuration.has_robot_filename())
      << "Robot config missing from Protocol Buffer:" << std::endl
      << configuration.DebugString();
  CHECK(configuration.has_environment_filename())
      << "Environment missing from Protocol Buffer:" << std::endl
      << configuration.DebugString();
  boost::filesystem::path robot_file_path(config_file_path);
  Robot robot = configuration.robot_config();
  if (configuration.has_robot_filename()) {
    robot_file_path /= configuration.robot_filename();
    std::ifstream fin(robot_file_path.native());
    std::string input_string((std::istreambuf_iterator<char>(fin)),
                             std::istreambuf_iterator<char>());
    bool success = google::protobuf::TextFormat::ParseFromString(
        input_string, &robot);
    fin.close();
    CHECK(success) << "Failed parsing file: " << robot_file_path.native();
  }
  robot_file_path.remove_filename();

  std::vector<int> parts_per_joint;
  std::vector<std::string> parts;
  for (const Segment& item : robot.segments()) {
    parts_per_joint.push_back(item.parts_size());
    for (const std::string& part : item.parts())
      parts.push_back((robot_file_path / part).native());
  }

  const std::string& environment = configuration.environment_filename();
  double max_underestimate = configuration.max_underestimate();

  part_count_ = parts.size();
  variance_ = max_underestimate;
  is_joint_start_.resize(parts.size(), false);

  LoadDhAndRanges(robot);

  int part_index = 0;
  int segment_index = -1;
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
    fcl::Vec3f v = dh_parameters_[segment_index]->getTranslation();
    v.normalize();
    double x = v[0];
    double y = v[1];
    double z = v[2];
    part_meshes_.emplace_back(ParseModel(
        part, dh_inverted_[segment_index].get(), x, y, z, &l, &r));
    parts_.emplace_back(new fcl::CollisionObject(
        part_meshes_.back(), fcl::Transform3f()));
    cylinders_.emplace_back(fcl::Vec3f(x * l, y * l, z * l), r);
    part_index++;
  }
  double l, r;
  environment_mesh_.reset(ParseModel(
      (config_file_path / environment).native(), dh_inverted_[0].get(),
      0.0, 0.0, 0.0, &l, &r));
  environment_.reset(new fcl::CollisionObject(
      environment_mesh_, fcl::Transform3f()));
}

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
