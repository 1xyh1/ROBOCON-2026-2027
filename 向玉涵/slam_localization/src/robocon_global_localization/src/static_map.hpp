#pragma once

#include "simple_pgo.h"

struct MapObservation {
  std::size_t anchor_id;
  M3D anchor_from_body_rotation;
  V3D anchor_from_body_translation;
  CloudType::ConstPtr cloud;
};

struct StaticMap {
  CloudType::Ptr cloud;
  std::size_t occupied_voxels;
  std::size_t removed_voxels;
  std::size_t input_points;
  std::size_t removed_points;
};

StaticMap buildStaticMap(const std::vector<KeyPoseWithCloud> &keyframes,
                         const std::vector<MapObservation> &observations,
                         const V3D &lidar_origin_body, float map_resolution);
