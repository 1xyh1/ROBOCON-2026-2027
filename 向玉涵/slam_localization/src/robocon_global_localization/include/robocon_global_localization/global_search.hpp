#pragma once

#include <Eigen/Core>
#include <chrono>
#include <memory>
#include <vector>

namespace cpu { class BBS3D; }

namespace robocon_global_localization {

// Map coordinates use +Z up. up_in_scan is the stationary IMU specific-force
// direction in the same body frame as the deskewed scan.
struct CoarseSearchResult {
  std::vector<Eigen::Matrix4f> seeds;
  const char *rejection_reason = nullptr;
};

class GlobalSearch {
 public:
  GlobalSearch(const std::vector<Eigen::Vector3d> &map, double resolution_m, double fine_basin_m);
  void use_loop_search_budget(double minimum_score);
  ~GlobalSearch();
  CoarseSearchResult propose(const std::vector<Eigen::Vector3d> &scan,
                             const Eigen::Vector3d &up_in_scan,
                             std::chrono::steady_clock::time_point deadline);
  std::vector<Eigen::Matrix4f> search(const std::vector<Eigen::Vector3d> &scan,
                        const Eigen::Vector3d &up_in_scan,
                        std::chrono::steady_clock::time_point deadline);

 private:
  std::unique_ptr<cpu::BBS3D> search_;
};

}  // namespace robocon_global_localization
