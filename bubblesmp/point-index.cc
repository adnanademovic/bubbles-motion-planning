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

#include "point-index.h"

#include <gflags/gflags.h>

DEFINE_bool(point_index_force_flann, false,
    "Sets if point indices should have to use flann");
DEFINE_bool(point_index_force_simple, false,
    "Sets if point indices should have to use a simple index "
    "(faster for small amounts of points)");

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace {
double DistanceSquared(
    const std::vector<double>& first, const std::vector<double>& second) {
  double d = 0.0;
  double c = 0.0;
  size_t l = first.size();
  for (size_t i = 0; i < l; ++i) {
    c = second[i] - first[i];
    d += c * c;
  }
  return d;
}
}  // namespace

PointIndex::PointIndex(const std::vector<double>& q_root, TreeNode* root_node)
    : PointIndex(q_root, root_node, false, IndexSettings()) {}

PointIndex::PointIndex(const std::vector<double>& q_root, TreeNode* root_node,
    const IndexSettings& flann_settings)
    : PointIndex(q_root, root_node, false, flann_settings) {}

PointIndex::PointIndex(const std::vector<double>& q_root, TreeNode* root_node,
                       bool using_flann_index, const IndexSettings& settings)
    : using_flann_index_(FLAGS_point_index_force_flann ||
          (using_flann_index && !FLAGS_point_index_force_simple)),
      attachment_points_(1, AttachmentPoint{q_root, root_node}),
      root_node_(root_node) {
  if (using_flann_index_) {
    if (settings.has_search_params()) {
      const SearchParams& pb_search_params = settings.search_params();
      search_parameters_.checks = pb_search_params.checks();
      search_parameters_.eps = pb_search_params.eps();
      search_parameters_.checks = pb_search_params.checks();
      search_parameters_.max_neighbors = pb_search_params.max_neighbors();
      if (pb_search_params.has_use_heap()) {
        if (pb_search_params.use_heap())
          search_parameters_.use_heap = flann::FLANN_True;
        else
          search_parameters_.use_heap = flann::FLANN_False;
      } else
          search_parameters_.use_heap = flann::FLANN_Undefined;
      search_parameters_.cores = pb_search_params.cores();
      search_parameters_.matrices_in_gpu_ram =
          pb_search_params.matrices_in_gpu_ram();
    }

    flann::IndexParams index_params;
    IndexSettings::Type index_settings_type = IndexSettings::AUTOTUNED;
    if (settings.has_type())
      index_settings_type = settings.type();
    IndexParams pb_index_params;
    if (settings.has_index_params())
      pb_index_params = settings.index_params();

    flann::flann_centers_init_t centers_init;
    if (pb_index_params.has_centers_init())
      centers_init = static_cast<flann::flann_centers_init_t>(
          pb_index_params.centers_init());

    switch (index_settings_type) {
      case IndexSettings::LINEAR:
        index_params = flann::LinearIndexParams();
        break;
      case IndexSettings::KD_TREE:
        index_params = flann::KDTreeIndexParams(pb_index_params.trees());
        break;
      case IndexSettings::KD_TREE_SINGLE:
        index_params = flann::KDTreeSingleIndexParams(
            pb_index_params.leaf_max_size());
        break;
      case IndexSettings::K_MEANS:
        index_params = flann::KMeansIndexParams(
            pb_index_params.branching(), pb_index_params.iterations(),
            centers_init, pb_index_params.cb_index());
        break;
      case IndexSettings::COMPOSITE:
        index_params = flann::CompositeIndexParams(
            pb_index_params.trees(),
            pb_index_params.branching(), pb_index_params.iterations(),
            centers_init, pb_index_params.cb_index());
        break;
      case IndexSettings::LSH:
        index_params = flann::LshIndexParams(
            pb_index_params.table_number(), pb_index_params.key_size(),
            pb_index_params.multi_probe_level());
        break;
      case IndexSettings::AUTOTUNED:
        index_params = flann::AutotunedIndexParams(
            pb_index_params.target_precision(), pb_index_params.build_weight(),
            pb_index_params.memory_weight(), pb_index_params.sample_fraction());
        break;
    }

    index_.reset(new flann::Index<flann::L2<double> >(flann::Matrix<double>(
        std::vector<double>(q_root).data(), 1, q_root.size()), index_params));

    index_->buildIndex();
  }
}

void PointIndex::AddPoint(const std::vector<double>& q, TreeNode* parent) {
  attachment_points_.push_back(AttachmentPoint{q, parent});
  if (using_flann_index_)
    index_->addPoints(
        flann::Matrix<double>(std::vector<double>(q).data(), 1, q.size()));
}

AttachmentPoint PointIndex::GetNearestPoint(
    const std::vector<double>& q) const {
  if (using_flann_index_) {
    std::vector<std::vector<int> > indices(1, std::vector<int>(1, 0));
    std::vector<std::vector<double> > distances(1, std::vector<double>(1, 0.0));
    index_->knnSearch(
        flann::Matrix<double>(std::vector<double>(q).data(), 1, q.size()),
        indices, distances, 1, search_parameters_);
    return attachment_points_[indices[0][0]];
  } else {
    double closest_distance = DistanceSquared(
        attachment_points_[0].position, q);
    int closest_point = 0;
    double dist_now;
    for (size_t i = attachment_points_.size() - 1; i > 0; --i) {
      dist_now = DistanceSquared(attachment_points_[i].position, q);
      if (dist_now < closest_distance) {
        closest_point = i;
        closest_distance = dist_now;
      }
    }
    return attachment_points_[closest_point];
  }
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
