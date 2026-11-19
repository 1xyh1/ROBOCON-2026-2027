#include "robocon_global_localization/gicp_matcher.hpp"
#include "robocon_global_localization/global_search.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <chrono>
#include <atomic>
#include <random>

namespace rgl = robocon_global_localization;

static rgl::Cloud::Ptr fixture(bool planar = false, float shift_x = 0.0f) {
  auto cloud = rgl::Cloud::Ptr(new rgl::Cloud);
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 7; ++y) {
      for (int z = 0; z < (planar ? 1 : 5); ++z) {
        pcl::PointXYZ point;
        point.x = shift_x + x * 0.08f + y * 0.003f;
        point.y = y * 0.09f + z * 0.002f;
        point.z = planar ? 0.0f : z * 0.11f + x * 0.001f;
        cloud->push_back(point);
      }
    }
  }
  return cloud;
}

static rgl::MatchConfig config() {
  rgl::MatchConfig value;
  value.voxel_m = 0.02f;
  value.minimum_points = 30;
  value.maximum_rmse_m = 0.05f;
  return value;
}

static auto deadline() { return std::chrono::steady_clock::now() + std::chrono::seconds(10); }

TEST(GicpMatcher, AcceptsUniqueThreeDimensionalMap) {
  const auto map = fixture();
  const rgl::GicpMatcher matcher(map, config());
  const auto result = matcher.match(map, {Eigen::Matrix4f::Identity()}, deadline());
  EXPECT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_LT(result.rmse_m, 0.01f);
  EXPECT_GT(result.overlap, 0.9f);
}

TEST(GicpMatcher, RejectsPlanarGeometry) {
  const auto map = fixture(true);
  const rgl::GicpMatcher matcher(map, config());
  const auto result = matcher.match(map, {Eigen::Matrix4f::Identity()}, deadline());
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.rejection_reason, "weak_3d_geometry");
}

TEST(GicpMatcher, RejectsEquivalentSeparateMapRegions) {
  const auto scan = fixture();
  const auto map = fixture();
  *map += *fixture(false, 3.0f);
  const rgl::GicpMatcher matcher(map, config());
  Eigen::Matrix4f second = Eigen::Matrix4f::Identity();
  second(0, 3) = 3.0f;
  const auto result = matcher.match(scan, {Eigen::Matrix4f::Identity(), second}, deadline());
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.ambiguous) << result.rejection_reason;
  EXPECT_EQ(result.evaluated_seeds, 2u);
}

TEST(GicpMatcher, PrefersCompleteNoisyMatchToExactPartialAlias) {
  const auto scan = rgl::Cloud::Ptr(new rgl::Cloud);
  const auto map = rgl::Cloud::Ptr(new rgl::Cloud);
  int index = 0;
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 5; ++y) {
      for (int z = 0; z < 5; ++z, ++index) {
        pcl::PointXYZ point;
        point.x = x * 0.5f;
        point.y = y * 0.5f;
        point.z = z * 0.5f;
        scan->push_back(point);
        auto correct = point;
        correct.z += index % 2 == 0 ? 0.02f : -0.02f;
        map->push_back(correct);
        if (index % 20 < 11) {
          point.x += 20.0f;
          map->push_back(point);
        }
      }
    }
  }
  Eigen::Matrix4f alias = Eigen::Matrix4f::Identity();
  alias(0, 3) = 20.0f;
  const rgl::GicpMatcher matcher(map, config());
  const auto result = matcher.match(scan, {Eigen::Matrix4f::Identity(), alias}, deadline());
  ASSERT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_LT((result.map_from_scan.block<3, 1>(0, 3)).norm(), 0.05f);
  EXPECT_GT(result.overlap, 0.95f);
}

TEST(GicpMatcher, RejectsDeadlineAndMissingSeeds) {
  const auto map = fixture();
  const rgl::GicpMatcher matcher(map, config());
  EXPECT_FALSE(matcher.match(map, {}, deadline()).accepted);
  const auto expired = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const auto result = matcher.match(map, {Eigen::Matrix4f::Identity()}, expired);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.rejection_reason, "matching_timeout");
}

TEST(GicpMatcher, DoesNotAcceptAlreadyCanceledRequest) {
  const auto map = fixture();
  const rgl::GicpMatcher matcher(map, config());
  std::atomic<bool> canceled{true};
  const auto result = matcher.match(map, {Eigen::Matrix4f::Identity()}, deadline(), &canceled);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.rejection_reason, "cancelled");
  EXPECT_EQ(result.evaluated_seeds, 0u);
}

TEST(GlobalSearch, FindsArbitraryPoseThenGicpRefinesWithoutInitialPose) {
  std::mt19937 generator(20261103);
  std::uniform_real_distribution<double> random(-1.0, 1.0);
  std::vector<Eigen::Vector3d> map_points;
  auto map = rgl::Cloud::Ptr(new rgl::Cloud);
  for (int i = 0; i < 500; ++i) {
    const Eigen::Vector3d p(3.0 * random(generator), 2.0 * random(generator),
                            1.3 + random(generator));
    map_points.push_back(p);
    map->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
  }
  rgl::GlobalSearch global(map_points, 0.1, 0.35);
  auto settings = config();
  settings.voxel_m = 0.1f;
  const rgl::GicpMatcher matcher(map, settings);
  for (const double yaw : {2.1467549799530254, -2.3911010752322315}) {
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-0.21, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.34, Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d translation(yaw > 0.0 ? 1.4 : -1.3, -0.7, 0.6);
    std::vector<Eigen::Vector3d> scan_points;
    auto scan = rgl::Cloud::Ptr(new rgl::Cloud);
    for (int i = 0; i < 350; ++i) {
      const Eigen::Vector3d p = rotation.transpose() * (map_points[i] - translation);
      scan_points.push_back(p);
      scan->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
    const auto coarse = global.search(scan_points,
        rotation.transpose() * Eigen::Vector3d::UnitZ(), deadline());
    const auto result = matcher.match(scan, coarse, deadline());
    ASSERT_TRUE(result.accepted) << result.rejection_reason;
    EXPECT_LT((result.map_from_scan.block<3, 1>(0, 3).cast<double>() - translation).norm(), 0.02);
    const Eigen::Matrix3d delta = result.map_from_scan.block<3, 3>(0, 0).cast<double>() * rotation.transpose();
    EXPECT_LT(Eigen::AngleAxisd(delta).angle(), 0.01);
  }
}

TEST(GlobalSearch, ExposesRepeatedRegionsForGicpAmbiguityCheck) {
  std::mt19937 generator(20261103);
  std::uniform_real_distribution<double> random(-1.0, 1.0);
  std::vector<Eigen::Vector3d> map_points, scan_points;
  auto map = rgl::Cloud::Ptr(new rgl::Cloud);
  auto scan = rgl::Cloud::Ptr(new rgl::Cloud);
  for (int i = 0; i < 500; ++i) {
    const Eigen::Vector3d p(3.0 * random(generator), 2.0 * random(generator), 1.3 + random(generator));
    for (const double offset : {0.0, 10.0}) {
      map_points.push_back(p + Eigen::Vector3d(offset, 0.0, 0.0));
      map->push_back(pcl::PointXYZ(p.x() + offset, p.y(), p.z()));
    }
    if (i < 350) {
      scan_points.push_back(p);
      scan->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
  }
  rgl::GlobalSearch global(map_points, 0.1, 0.35);
  const auto seeds = global.search(scan_points, Eigen::Vector3d::UnitZ(), deadline());
  ASSERT_GE(seeds.size(), 2u);
  auto settings = config();
  settings.voxel_m = 0.1f;
  const rgl::GicpMatcher matcher(map, settings);
  const auto result = matcher.match(scan, seeds, deadline());
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.ambiguous) << result.rejection_reason;
}
