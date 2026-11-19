#include "robocon_global_localization/global_search.hpp"

#include <cpu_bbs3d/bbs3d.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace robocon_global_localization {

GlobalSearch::GlobalSearch(const std::vector<Eigen::Vector3d> &map, double resolution_m, double fine_basin_m)
    : search_(std::make_unique<cpu::BBS3D>()) {
  search_->set_tar_points(map, resolution_m, 6);
  search_->set_trans_search_range(map);
  search_->set_angular_search_range(Eigen::Vector3d::Zero(),
                                    Eigen::Vector3d(0.0, 0.0, 2.0 * std::acos(-1.0)));
  search_->enable_timeout();
  search_->set_candidate_separation(2.0 * fine_basin_m);
}

GlobalSearch::~GlobalSearch() = default;

void GlobalSearch::use_loop_search_budget(double minimum_score) {
  // Mapping runs the proposal in parallel with the odometry callbacks.
  search_->set_num_threads(1);
  search_->set_score_threshold_percentage(minimum_score);
}

CoarseSearchResult GlobalSearch::propose(const std::vector<Eigen::Vector3d> &scan,
                                   const Eigen::Vector3d &up_in_scan,
                                   std::chrono::steady_clock::time_point deadline) {
  const Eigen::Matrix3d upright = Eigen::Quaterniond::FromTwoVectors(
      up_in_scan, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::vector<Eigen::Vector3d> aligned(scan.size());
  for (std::size_t i = 0; i < scan.size(); ++i) aligned[i] = upright * scan[i];
  const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
  if (remaining <= 0) return {{}, "global_search_timeout"};
  search_->set_timeout_duration_in_msec(static_cast<int>(remaining));
  search_->set_src_points(aligned);
  search_->localize();
  if (search_->has_timed_out()) return {{}, "global_search_timeout"};
  if (search_->has_multiple_regions()) return {{}, "global_search_multiple_regions"};
  if (!search_->has_localized()) return {{}, "global_search_no_match"};
  Eigen::Matrix4d scan_to_upright = Eigen::Matrix4d::Identity();
  scan_to_upright.block<3, 3>(0, 0) = upright;
  std::vector<Eigen::Matrix4f> result;
  for (const auto &pose : search_->get_pose_candidates()) result.push_back((pose * scan_to_upright).cast<float>());
  return {std::move(result), nullptr};
}

std::vector<Eigen::Matrix4f> GlobalSearch::search(const std::vector<Eigen::Vector3d> &scan,
                                   const Eigen::Vector3d &up_in_scan,
                                   std::chrono::steady_clock::time_point deadline) {
  auto result = propose(scan, up_in_scan, deadline);
  if (result.rejection_reason) throw std::runtime_error(result.rejection_reason);
  return std::move(result.seeds);
}

}  // namespace robocon_global_localization
