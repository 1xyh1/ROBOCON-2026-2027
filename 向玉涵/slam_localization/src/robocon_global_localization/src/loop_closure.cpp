#include "simple_pgo.h"
#include "robocon_global_localization/gicp_matcher.hpp"
#include "robocon_global_localization/global_search.hpp"

#include <pcl/common/io.h>
#include <algorithm>
#include <chrono>

namespace rgl = robocon_global_localization;
namespace {
std::vector<Eigen::Vector3d> coarse_points(const rgl::Cloud::ConstPtr &cloud, float resolution) {
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(resolution, resolution, resolution);
  rgl::Cloud reduced;
  filter.filter(reduced);
  std::vector<Eigen::Vector3d> result;
  result.reserve(reduced.size());
  for (const auto &point : reduced) result.emplace_back(point.x, point.y, point.z);
  return result;
}
}  // namespace

LoopSearchResult findLoop(const std::vector<KeyPoseWithCloud> &frames, std::size_t history_count,
                          double map_resolution) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  auto target = rgl::Cloud::Ptr(new rgl::Cloud);
  for (std::size_t i = 0; i < history_count; ++i) {
    const auto &key = frames[i];
    const auto rotation = key.r_global.cast<float>().eval();
    const auto translation = key.t_global.cast<float>().eval();
    for (const auto &point : *key.body_cloud) {
      const Eigen::Vector3f world = rotation * point.getVector3fMap() + translation;
      target->emplace_back(world.x(), world.y(), world.z());
    }
  }
  const auto &current = frames.back();
  auto source = rgl::Cloud::Ptr(new rgl::Cloud);
  pcl::copyPointCloud(*current.body_cloud, *source);
  rgl::MatchConfig settings;
  settings.voxel_m = map_resolution;
  settings.max_correspondence_m = 0.50f;
  settings.minimum_observability_ratio = 0.01f;
  const auto minimum_points = static_cast<std::size_t>(settings.minimum_points);
  const auto fine_target = coarse_points(target, settings.voxel_m);
  if (fine_target.size() < minimum_points) return {{}, "insufficient_history_points"};

  const float coarse_resolution = 3.0f * settings.voxel_m;
  const auto map_points = coarse_points(target, coarse_resolution);
  const auto query_points = coarse_points(source, coarse_resolution);
  rgl::GlobalSearch global(map_points, coarse_resolution, settings.max_correspondence_m);
  global.use_loop_search_budget(0.25);
  const auto coarse = global.propose(query_points, current.r_local.transpose() * V3D::UnitZ(),
                                    deadline - std::chrono::seconds(1));
  if (coarse.rejection_reason) return {{}, coarse.rejection_reason};

  const rgl::GicpMatcher matcher(target, settings);
  const auto match = matcher.match(source, coarse.seeds, deadline);
  if (!match.accepted) return {{}, match.rejection_reason};
  if (match.evaluated_seeds != coarse.seeds.size() || std::chrono::steady_clock::now() >= deadline) {
    return {{}, "loop_verification_timeout"};
  }

  // Anchor the verified global pose to an older keyframe. A loop factor
  // remains relative while new odometry keys are appended during the search.
  const V3D t = match.map_from_scan.block<3, 1>(0, 3).cast<double>();
  const M3D r = match.map_from_scan.block<3, 3>(0, 0).cast<double>();
  const auto end = frames.begin() + history_count;
  const auto anchor = std::min_element(frames.begin(), end, [&](const auto &a, const auto &b) {
    return (a.t_global - t).squaredNorm() < (b.t_global - t).squaredNorm();
  });
  const auto anchor_id = static_cast<std::size_t>(anchor - frames.begin());
  const double variance = std::max(1e-6, static_cast<double>(match.rmse_m * match.rmse_m));
  return {LoopPair{frames.size() - 1, anchor_id, anchor->r_global.transpose() * r,
                   anchor->r_global.transpose() * (t - anchor->t_global), variance,
                   match.rmse_m, match.overlap}, "accepted"};
}
