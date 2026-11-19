#include "../src/static_map.hpp"
#include "robocon_global_localization/gicp_matcher.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>

namespace {
PointType point(float x, float y, float z, float intensity = 1.0f) {
  PointType p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

CloudType::Ptr room() {
  auto scan = CloudType::Ptr(new CloudType);
  for (int x = -17; x <= 13; ++x) {
    for (int y = -10; y <= 14; ++y) {
      scan->push_back(point(0.21f * x, 0.19f * y, -1.13f));
    }
  }
  for (int z = -4; z <= 9; ++z) {
    for (int y = -10; y <= 14; ++y) scan->push_back(point(-3.72f, 0.19f * y, 0.23f * z));
    for (int x = -12; x <= 13; ++x) scan->push_back(point(0.21f * x, 2.87f, 0.23f * z));
    for (int y = -7; y <= -3; ++y) scan->push_back(point(1.37f, 0.16f * y, 0.20f * z));
  }
  return scan;
}

CloudWithPose frame(const CloudType::Ptr &scan, int time, const V3D &translation,
                    const M3D &rotation = M3D::Identity()) {
  CloudWithPose input;
  input.pose.setTime(time, 0);
  input.pose.r = rotation;
  input.pose.t = translation;
  input.cloud = scan;
  return input;
}
}  // namespace

TEST(PoseGraph, GeometricLoopCorrectsDriftBeyondOldSearchRadius) {
  Config config;
  config.key_pose_delta_trans = 0.5;
  config.loop_time_tresh = 20.5;  // Only the first keyframe is historical at the revisit.
  SimplePGO graph(config);
  const auto scene = room();
  const double pi = std::acos(-1.0);
  const V3D drift(2.15, -0.2, 0.0);
  for (int i = 0; i <= 20; ++i) {
    const double angle = i * 2.0 * pi / 20.0;
    const V3D truth(2.0 - 2.0 * std::cos(angle), 2.0 * std::sin(angle), 0.0);
    const M3D rotation = Eigen::AngleAxisd(i == 20 ? 137.0 * pi / 180.0 : angle, V3D::UnitZ()).toRotationMatrix();
    auto scan = CloudType::Ptr(new CloudType);
    for (const auto &world : *scene) {
      const V3D body = rotation.transpose() * (world.getVector3fMap().cast<double>() - truth);
      scan->push_back(point(body.x(), body.y(), body.z()));
    }
    ASSERT_TRUE(graph.addKeyPose(frame(scan, 1000 + i + (i == 20), truth + i * drift / 20.0, rotation)));
    graph.smoothAndUpdate();
  }
  graph.searchForLoopPairs();
  const auto result = graph.collectLoop(true);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->pair) << result->reason;
  EXPECT_EQ(result->pair->target_id, 0u);
  EXPECT_EQ(result->pair->source_id, 20u);
  graph.smoothAndUpdate();
  const double optimized = graph.keyPoses().back().t_global.norm();
  std::cout << "closure_error_before_m=" << drift.norm() << " closure_error_after_m=" << optimized
            << " yaw_revisit_deg=137 overlap=" << result->pair->overlap << '\n';
  EXPECT_LT(optimized, 0.25);
}

TEST(PoseGraph, CompetingIdenticalRoomsDoNotEnterTheGraph) {
  Config config;
  config.loop_time_tresh = 10.0;
  SimplePGO graph(config);
  const auto scene = room();
  ASSERT_TRUE(graph.addKeyPose(frame(scene, 1000, V3D::Zero())));
  graph.smoothAndUpdate();
  ASSERT_TRUE(graph.addKeyPose(frame(scene, 1001, V3D(12, 0, 0))));
  graph.smoothAndUpdate();
  ASSERT_TRUE(graph.addKeyPose(frame(scene, 1018, V3D(9, 0, 0))));
  graph.smoothAndUpdate();
  ASSERT_TRUE(graph.addKeyPose(frame(scene, 1020, V3D(3, 0, 0))));
  graph.smoothAndUpdate();
  graph.searchForLoopPairs();
  const auto result = graph.collectLoop(true);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->pair);
  EXPECT_TRUE(graph.historyPairs().empty());
  EXPECT_EQ(result->reason, "multiple_comparable_poses");
}

TEST(PoseGraph, CorridorFitDoesNotConstrainTranslationAlongTheCorridor) {
  namespace rgl = robocon_global_localization;
  auto corridor = rgl::Cloud::Ptr(new rgl::Cloud);
  auto scan = rgl::Cloud::Ptr(new rgl::Cloud);
  for (int x = -35; x <= 35; ++x) {
    const float forward = x * 0.21f;
    for (int j = 0; j < 12; ++j) {
      const auto floor = pcl::PointXYZ(forward, (j - 5.5f) * 0.2f, -1.4f);
      const auto left = pcl::PointXYZ(forward, -1.7f, j * 0.21f - 0.65f);
      const auto right = pcl::PointXYZ(forward, 1.7f, j * 0.21f - 0.65f);
      for (const auto &p : {floor, left, right}) {
        corridor->push_back(p);
        if (std::abs(x) < 20) scan->push_back(p);
      }
    }
  }
  rgl::MatchConfig config;
  config.minimum_observability_ratio = 0.01f;
  rgl::GicpMatcher matcher(corridor, config);
  const auto result = matcher.match(scan, {M4F::Identity()},
                                     std::chrono::steady_clock::now() + std::chrono::seconds(3));
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.rejection_reason, "weak_loop_observability");
  EXPECT_GT(result.overlap, 0.95f);
}

TEST(StaticMap, ClearsTwoMovingClustersAndKeepsWallsAndOccludedPoints) {
  KeyPoseWithCloud key;
  key.r_global = Eigen::AngleAxisd(0.39, V3D::UnitZ()).toRotationMatrix();
  key.t_global = V3D(2.1, -1.7, 0.48);
  const V3D optical_origin(-0.011, -0.15, 0.045);
  std::vector<V3D> movers;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      movers.emplace_back(1.43, 0.42 + 0.29 * i, 0.36 + 0.27 * j);
      movers.emplace_back(2.29, -1.23 + 0.29 * i, -0.37 + 0.27 * j);
    }
  }
  const V3D occluded(4.21, -0.31, 0.62);
  std::vector<MapObservation> observations;
  for (int i = 0; i < 7; ++i) {
    const V3D shift(0, 0.015 * i, 0);
    const V3D origin = optical_origin + shift;
    auto scan = CloudType::Ptr(new CloudType);
    const auto add = [&](const V3D &p, float intensity) {
      const V3D body = p - shift;
      scan->push_back(point(body.x(), body.y(), body.z(), intensity));
    };
    for (const auto &actor : movers) {
      if (i == 0) add(actor, 100.0f);
      else add(origin + (actor - origin) * ((5.51 - origin.x()) / (actor.x() - origin.x())), 1.0f);
    }
    add(V3D(5.51, 0.1, -1.31), 1.0f);
    add(V3D(5.51, 0.1, -1.14), 1.0f);
    add(V3D(2.32, -2.83, 0.35), 4.0f);  // thin fixed support, outside the moving clusters
    if (i == 0) add(occluded, 7.0f);
    else add(origin + 0.58 * (occluded - origin), 8.0f);
    observations.push_back({0, M3D::Identity(), shift, scan});
  }
  const auto map = buildStaticMap({key}, observations, optical_origin, 0.10f);
  std::cout << "moving_labeled=" << movers.size() << " removed_returns=" << map.removed_points
            << " removed_voxels=" << map.removed_voxels << " static_points=" << map.cloud->size() << '\n';
  EXPECT_EQ(map.removed_points, movers.size());
  EXPECT_TRUE(std::none_of(map.cloud->begin(), map.cloud->end(), [](const auto &p) { return p.intensity > 10; }));
  for (float expected : {1.0f, 4.0f, 7.0f, 8.0f}) {
    EXPECT_TRUE(std::any_of(map.cloud->begin(), map.cloud->end(), [&](const auto &p) { return p.intensity == expected; }));
  }
}

TEST(StaticMap, ManyRaysFromOneScanDoNotMakeSeveralFreeObservations) {
  KeyPoseWithCloud key;
  key.r_global = M3D::Identity();
  key.t_global = V3D::Zero();
  auto occupied = CloudType::Ptr(new CloudType);
  occupied->push_back(point(1.51, 0.51, 0.51));
  auto one_clear_scan = CloudType::Ptr(new CloudType);
  for (int i = 0; i < 30; ++i) one_clear_scan->push_back(point(3.02 + 0.001f * i, 1.02, 1.02));
  auto looking_elsewhere = CloudType::Ptr(new CloudType);
  looking_elsewhere->push_back(point(-4.0, -1.0, 1.0));
  const std::vector<MapObservation> observations = {
      {0, M3D::Identity(), V3D::Zero(), occupied},
      {0, M3D::Identity(), V3D::Zero(), one_clear_scan},
      {0, M3D::Identity(), V3D::Zero(), looking_elsewhere},
      {0, M3D::Identity(), V3D::Zero(), looking_elsewhere}};
  const auto map = buildStaticMap({key}, observations, V3D::Zero(), 0.10f);
  EXPECT_EQ(map.removed_voxels, 0u);
}
