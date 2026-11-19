#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace robocon_global_localization {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct MatchConfig {
  float voxel_m = 0.10f;
  float max_correspondence_m = 0.35f;
  float overlap_distance_m = 0.20f;
  float maximum_rmse_m = 0.08f;
  float minimum_overlap = 0.55f;
  float minimum_3d_geometry_ratio = 0.003f;
  int minimum_points = 80;
  int maximum_iterations = 35;
  float ambiguity_translation_m = 0.10f;
  float ambiguity_rotation_rad = 0.0872665f;
  float ambiguity_rmse_margin_m = 0.005f;
  float ambiguity_overlap_margin = 0.05f;
  float minimum_observability_ratio = 0.0f;
};

struct MatchResult {
  bool accepted = false;
  bool converged = false;
  bool ambiguous = false;
  std::string rejection_reason;
  Eigen::Matrix4f map_from_scan = Eigen::Matrix4f::Identity();
  float rmse_m = 0.0f;
  float overlap = 0.0f;
  std::array<double, 3> geometry_eigenvalues = {0.0, 0.0, 0.0};
  std::size_t evaluated_seeds = 0;
};

class GicpMatcher {
 public:
  GicpMatcher(const Cloud::ConstPtr &map, MatchConfig config = {});
  MatchResult match(const Cloud::ConstPtr &scan, const std::vector<Eigen::Matrix4f> &seeds,
                    std::chrono::steady_clock::time_point deadline,
                    const std::atomic<bool> *cancel_requested = nullptr) const;

 private:
  struct Target;
  std::shared_ptr<const Target> target_;
  MatchConfig config_;
};

}  // namespace robocon_global_localization
