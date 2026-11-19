#include "robocon_global_localization/gicp_matcher.hpp"

#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/impl/gicp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace robocon_global_localization {
namespace {

Cloud::Ptr downsample(const Cloud::ConstPtr &input, float voxel_m) {
  auto result = Cloud::Ptr(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(voxel_m, voxel_m, voxel_m);
  filter.filter(*result);
  return result;
}

double rotation_distance(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b) {
  const Eigen::Matrix3f relative = a.block<3, 3>(0, 0) * b.block<3, 3>(0, 0).transpose();
  const double cosine = std::clamp((static_cast<double>(relative.trace()) - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine);
}

// Spatial spread of returns does not constrain translation along a corridor.
double observability_ratio(const Cloud &map, const pcl::search::KdTree<pcl::PointXYZ> &tree,
                           const std::vector<Eigen::Vector3d> &points,
                           const Eigen::Vector3d &center, double lever_arm) {
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  std::vector<int> neighbors;
  std::vector<float> squared;
  std::size_t normal_count = 0;
  for (const auto &point : points) {
    if (tree.nearestKSearch(pcl::PointXYZ(point.x(), point.y(), point.z()), 12, neighbors, squared) < 12) continue;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int index : neighbors) mean += map[index].getVector3fMap().cast<double>();
    mean /= neighbors.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int index : neighbors) {
      const Eigen::Vector3d delta = map[index].getVector3fMap().cast<double>() - mean;
      covariance.noalias() += delta * delta.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> normals(covariance);
    const auto spectrum = normals.eigenvalues();
    if (spectrum[1] <= 0.01 * spectrum[2] || spectrum[0] > 0.1 * spectrum[1]) continue;
    const Eigen::Vector3d normal = normals.eigenvectors().col(0);
    Eigen::Matrix<double, 6, 1> jacobian;
    jacobian << (point - center).cross(normal) / lever_arm, normal;
    information.noalias() += jacobian * jacobian.transpose();
    ++normal_count;
  }
  if (normal_count < 30) return 0.0;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information, Eigen::EigenvaluesOnly);
  return solver.eigenvalues()[0] / solver.eigenvalues()[5];
}

struct Candidate {
  MatchResult result;
  bool eligible = false;
  double coverage_rmse_m = std::numeric_limits<double>::infinity();
};

class Gicp : public pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> {
 public:
  MatricesVectorPtr target_covariances(const Cloud::ConstPtr &cloud, const KdTreePtr &tree) {
    auto covariances = std::make_shared<MatricesVector>();
    computeCovariances<pcl::PointXYZ>(cloud, tree, *covariances);
    return covariances;
  }
};

}  // namespace

struct GicpMatcher::Target {
  explicit Target(Cloud::ConstPtr points)
      : map(std::move(points)), tree(std::make_shared<Gicp::KdTree>()) {
    tree->setInputCloud(map);
    covariances = Gicp().target_covariances(map, tree);
  }
  Cloud::ConstPtr map;
  Gicp::KdTreePtr tree;
  Gicp::MatricesVectorPtr covariances;
};

GicpMatcher::GicpMatcher(const Cloud::ConstPtr &map, MatchConfig config) : config_(config) {
  if (!map || map->empty() || config.voxel_m <= 0.0f || config.max_correspondence_m <= 0.0f ||
      config.minimum_points < 3 || config.maximum_iterations < 1 ||
      config.minimum_overlap <= 0.0f || config.minimum_overlap > 1.0f) {
    throw std::invalid_argument("invalid GICP map or configuration");
  }
  auto voxels = downsample(map, config.voxel_m);
  if (voxels->size() < static_cast<std::size_t>(config_.minimum_points)) {
    throw std::invalid_argument("static map has too few finite voxel points");
  }
  target_ = std::make_shared<Target>(std::move(voxels));
}

MatchResult GicpMatcher::match(const Cloud::ConstPtr &scan,
                              const std::vector<Eigen::Matrix4f> &seeds,
                              std::chrono::steady_clock::time_point deadline,
                              const std::atomic<bool> *cancel_requested) const {
  MatchResult rejected;
  rejected.rejection_reason = "no_scan_or_seed";
  rejected.rmse_m = std::numeric_limits<float>::infinity();
  if (cancel_requested && cancel_requested->load()) {
    rejected.rejection_reason = "cancelled";
    return rejected;
  }
  if (!scan || scan->empty() || seeds.empty()) return rejected;
  auto source = downsample(scan, config_.voxel_m);
  if (source->size() < static_cast<std::size_t>(config_.minimum_points)) {
    rejected.rejection_reason = "insufficient_scan_points";
    return rejected;
  }
  const auto &tree = *target_->tree;
  std::vector<Candidate> candidates;
  for (const auto &seed : seeds) {
    if (cancel_requested && cancel_requested->load()) {
      rejected.rejection_reason = "cancelled";
      rejected.evaluated_seeds = candidates.size();
      return rejected;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      rejected.rejection_reason = "matching_timeout";
      break;
    }
    if (!seed.allFinite()) continue;
    Gicp gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target_->map);
    gicp.setSearchMethodTarget(target_->tree, true);
    gicp.setTargetCovariances(target_->covariances);
    gicp.setMaximumIterations(config_.maximum_iterations);
    gicp.setMaxCorrespondenceDistance(config_.max_correspondence_m);
    gicp.setTransformationEpsilon(1e-5);
    Cloud aligned;
    gicp.align(aligned, seed);
    if (cancel_requested && cancel_requested->load()) {
      rejected.rejection_reason = "cancelled";
      rejected.evaluated_seeds = candidates.size() + 1;
      return rejected;
    }
    Candidate candidate;
    auto &result = candidate.result;
    result.evaluated_seeds = 1;
    result.converged = gicp.hasConverged();
    result.map_from_scan = gicp.getFinalTransformation();
    if (!result.converged || aligned.empty()) {
      result.rejection_reason = "gicp_not_converged";
      candidates.push_back(candidate);
      continue;
    }
    std::vector<Eigen::Vector3d> inliers;
    inliers.reserve(aligned.size());
    double squared_sum = 0.0;
    std::vector<int> indices(1);
    std::vector<float> squared(1);
    for (const auto &point : aligned) {
      if (!pcl::isFinite(point) || tree.nearestKSearch(point, 1, indices, squared) == 0 ||
          squared[0] > config_.overlap_distance_m * config_.overlap_distance_m) continue;
      squared_sum += squared[0];
      inliers.emplace_back(point.x, point.y, point.z);
    }
    result.overlap = static_cast<float>(inliers.size()) / static_cast<float>(aligned.size());
    result.rmse_m = inliers.empty() ? std::numeric_limits<float>::infinity()
                                   : static_cast<float>(std::sqrt(squared_sum / inliers.size()));
    // Compare hypotheses over the whole scan. An unmatched point contributes
    // the existing correspondence cutoff instead of disappearing from the cost.
    const double cutoff_squared = config_.overlap_distance_m * config_.overlap_distance_m;
    candidate.coverage_rmse_m = std::sqrt(
        (squared_sum + (aligned.size() - inliers.size()) * cutoff_squared) / aligned.size());
    if (inliers.size() < static_cast<std::size_t>(config_.minimum_points)) {
      result.rejection_reason = "insufficient_static_overlap";
    } else {
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      for (const auto &point : inliers) center += point;
      center /= static_cast<double>(inliers.size());
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (const auto &point : inliers) {
        const auto delta = point - center;
        covariance += delta * delta.transpose();
      }
      covariance /= static_cast<double>(inliers.size() - 1);
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
      const auto values = solver.eigenvalues();
      for (int i = 0; i < 3; ++i) result.geometry_eigenvalues[i] = values[i];
      if (values[2] <= 0.0 || values[0] / values[2] < config_.minimum_3d_geometry_ratio) {
        result.rejection_reason = "weak_3d_geometry";
      } else if (result.overlap < config_.minimum_overlap || result.rmse_m > config_.maximum_rmse_m) {
        result.rejection_reason = "poor_alignment";
      } else if (config_.minimum_observability_ratio > 0.0f &&
                 observability_ratio(*target_->map, tree, inliers, center, std::sqrt(values.sum())) <
                     config_.minimum_observability_ratio) {
        result.rejection_reason = "weak_loop_observability";
      } else {
        result.accepted = true;
        candidate.eligible = true;
      }
    }
    candidates.push_back(candidate);
  }
  if (cancel_requested && cancel_requested->load()) {
    rejected.rejection_reason = "cancelled";
    rejected.evaluated_seeds = candidates.size();
    return rejected;
  }
  if (candidates.empty()) return rejected;
  auto better = [](const Candidate &a, const Candidate &b) {
    if (a.eligible != b.eligible) return a.eligible;
    if (a.coverage_rmse_m != b.coverage_rmse_m) return a.coverage_rmse_m < b.coverage_rmse_m;
    return a.result.overlap > b.result.overlap;
  };
  const auto best = std::min_element(candidates.begin(), candidates.end(), better);
  MatchResult result = best->result;
  result.evaluated_seeds = candidates.size();
  if (result.accepted) {
    for (const auto &other : candidates) {
      if (!other.eligible || &other == &(*best)) continue;
      const auto distance = (other.result.map_from_scan.block<3, 1>(0, 3) -
                             result.map_from_scan.block<3, 1>(0, 3)).norm();
      const bool distinct = distance > config_.ambiguity_translation_m ||
                            rotation_distance(other.result.map_from_scan, result.map_from_scan) >
                                config_.ambiguity_rotation_rad;
      const bool comparable = other.coverage_rmse_m <= best->coverage_rmse_m + config_.ambiguity_rmse_margin_m &&
                              other.result.overlap >= result.overlap - config_.ambiguity_overlap_margin;
      if (distinct && comparable) {
        result.accepted = false;
        result.ambiguous = true;
        result.rejection_reason = "multiple_comparable_poses";
        break;
      }
    }
  }
  return result;
}

}  // namespace robocon_global_localization
