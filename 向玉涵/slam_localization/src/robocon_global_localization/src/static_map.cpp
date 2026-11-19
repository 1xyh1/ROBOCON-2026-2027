#include "static_map.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace {
using Cell = std::array<int, 3>;

struct CellHash {
  std::size_t operator()(const Cell &cell) const {
    return (static_cast<std::uint32_t>(cell[0]) * 73856093u) ^
           (static_cast<std::uint32_t>(cell[1]) * 19349663u) ^
           (static_cast<std::uint32_t>(cell[2]) * 83492791u);
  }
};

struct Evidence {
  V3F center = V3F::Zero();
  std::size_t points = 0;
  std::size_t hits = 0;
  std::size_t clears = 0;
  std::size_t hit_frame = std::numeric_limits<std::size_t>::max();
  std::size_t clear_frame = std::numeric_limits<std::size_t>::max();

  std::size_t requiredClears() const { return std::max<std::size_t>(3, 2 * hits + 1); }
  bool removable() const { return clears >= requiredClears(); }
};

using EvidenceGrid = std::unordered_map<Cell, Evidence, CellHash>;

struct MapFrame {
  M3F rotation;
  V3F translation;
  V3F origin;
  CloudType::ConstPtr cloud;

  V3F world(const PointType &point) const { return rotation * point.getVector3fMap() + translation; }
};

Cell cellAt(const V3F &point, float voxel) {
  return {static_cast<int>(std::floor(point.x() / voxel)),
          static_cast<int>(std::floor(point.y() / voxel)),
          static_cast<int>(std::floor(point.z() / voxel))};
}

bool adjacentHit(const EvidenceGrid &grid, Cell cell, std::size_t frame) {
  for (int axis = 0; axis < 3; ++axis) {
    for (int offset : {-1, 1}) {
      auto neighbor = cell;
      neighbor[axis] += offset;
      const auto found = grid.find(neighbor);
      if (found != grid.end() && found->second.hit_frame == frame) return true;
    }
  }
  return false;
}

// Amanatides-Woo traversal of the observed ray segment. Space beyond the
// return stays unknown. The final voxel-length guard covers small pose errors.
void observeFree(EvidenceGrid &grid, const V3F &origin, const V3F &hit,
                 float voxel, std::size_t frame, std::size_t frame_count) {
  const V3F delta = hit - origin;
  const float range = delta.norm();
  const float limit = range - voxel;
  if (limit <= 0.0f) return;
  const V3F direction = delta / range;
  Cell cell = cellAt(origin, voxel);
  std::array<float, 3> next, spacing;
  std::array<int, 3> step;
  for (int axis = 0; axis < 3; ++axis) {
    if (direction[axis] == 0.0f) {
      next[axis] = spacing[axis] = std::numeric_limits<float>::infinity();
      step[axis] = 0;
    } else {
      step[axis] = direction[axis] > 0.0f ? 1 : -1;
      const float edge = voxel * (cell[axis] + (step[axis] > 0));
      next[axis] = (edge - origin[axis]) / direction[axis];
      spacing[axis] = voxel / std::abs(direction[axis]);
    }
  }

  float entered = 0.0f;
  while (entered < limit) {
    const auto axis = std::min_element(next.begin(), next.end()) - next.begin();
    if (std::min(next[axis], limit) - entered >= 0.25f * voxel) {
      const auto found = grid.find(cell);
      if (found != grid.end()) {
        auto &e = found->second;
        if (e.hit_frame != frame && e.clear_frame != frame && !e.removable() &&
            e.hits + e.requiredClears() <= frame_count) {
          const V3F to_center = e.center - origin;
          const float projected = to_center.dot(direction);
          if (projected > 0.0f && projected < limit &&
              (to_center - projected * direction).squaredNorm() < 0.09f * voxel * voxel &&
              !adjacentHit(grid, cell, frame)) {
            ++e.clears;
            e.clear_frame = frame;
          }
        }
      }
    }
    entered = next[axis];
    next[axis] += spacing[axis];
    cell[axis] += step[axis];
  }
}
}  // namespace

StaticMap buildStaticMap(const std::vector<KeyPoseWithCloud> &keyframes,
                         const std::vector<MapObservation> &observations,
                         const V3D &lidar_origin_body, float map_resolution) {
  // Accumulate measured endpoints after the graph has updated the anchor poses.
  // Every return is an actual sample selected by UniformSampling upstream.
  std::vector<MapFrame> frames;
  frames.reserve(observations.size());
  for (const auto &obs : observations) {
    const auto &key = keyframes[obs.anchor_id];
    MapFrame frame;
    frame.rotation = (key.r_global * obs.anchor_from_body_rotation).cast<float>();
    frame.translation = (key.t_global + key.r_global * obs.anchor_from_body_translation).cast<float>();
    frame.origin = frame.translation + frame.rotation * lidar_origin_body.cast<float>();
    frame.cloud = obs.cloud;
    frames.push_back(std::move(frame));
  }

  const float voxel = 2.0f * map_resolution;
  EvidenceGrid grid;
  std::size_t input_points = 0;
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto &frame = frames[i];
    input_points += frame.cloud->size();
    for (const auto &point : *frame.cloud) {
      const V3F world = frame.world(point);
      auto &e = grid[cellAt(world, voxel)];
      e.center += world;
      ++e.points;
      if (e.hit_frame != i) {
        ++e.hits;
        e.hit_frame = i;
      }
    }
  }
  for (auto &[cell, e] : grid) e.center /= e.points;

  // Same-scan returns take precedence over free rays. Count at most one free
  // observation per sampled scan, and only in cells that had an actual hit.
  if (frames.size() >= 4) {
    for (std::size_t i = 0; i < frames.size(); ++i) {
      const auto &frame = frames[i];
      std::vector<V3F> endpoints;
      endpoints.reserve(frame.cloud->size());
      for (const auto &point : *frame.cloud) {
        const V3F world = frame.world(point);
        grid.at(cellAt(world, voxel)).hit_frame = i;
        endpoints.push_back(world);
      }
      for (const auto &endpoint : endpoints) observeFree(grid, frame.origin, endpoint, voxel, i, frames.size());
    }
  }

  const auto removed_voxels = static_cast<std::size_t>(std::count_if(
      grid.begin(), grid.end(), [](const auto &item) { return item.second.removable(); }));
  auto retained = CloudType::Ptr(new CloudType);
  std::size_t removed_points = 0;
  for (const auto &frame : frames) {
    for (const auto &point : *frame.cloud) {
      const V3F world = frame.world(point);
      if (grid.at(cellAt(world, voxel)).removable()) {
        ++removed_points;
      } else {
        PointType transformed = point;
        transformed.getVector3fMap() = world;
        retained->push_back(transformed);
      }
    }
  }
  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(retained);
  filter.setLeafSize(map_resolution, map_resolution, map_resolution);
  filter.filter(*retained);
  return {retained, grid.size(), removed_voxels, input_points, removed_points};
}
