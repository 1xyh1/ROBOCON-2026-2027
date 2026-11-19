#include "robocon_global_localization/gicp_matcher.hpp"
#include "robocon_global_localization/global_search.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robocon_interfaces/action/relocalize.hpp>
#include <robocon_interfaces/msg/localization_quality.hpp>
#include <robocon_interfaces/msg/time_alignment_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
namespace rgl = robocon_global_localization;

class GlobalLocalizer final : public rclcpp::Node {
  using Relocalize = robocon_interfaces::action::Relocalize;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Relocalize>;
  using Quality = robocon_interfaces::msg::LocalizationQuality;

  struct Snapshot {
    sensor_msgs::msg::PointCloud2::SharedPtr scan;
    nav_msgs::msg::Odometry::SharedPtr odom;
  };

 public:
  GlobalLocalizer() : Node("global_localizer") {
    const auto map_path = declare_parameter<std::string>("map_pcd", "");
    map_id_ = declare_parameter<std::string>("map_id", "");
    calibration_id_ = declare_parameter<std::string>("calibration_id", "");
    floor_ = declare_parameter<std::string>("floor", "");
    odom_epoch_ = declare_parameter<int64_t>("odom_epoch", -1);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    require_live_time_gate_ = declare_parameter<bool>("require_live_time_gate", true);
    require_live_odom_epoch_ = declare_parameter<bool>("require_live_odom_epoch", true);
    continuous_enabled_ = declare_parameter<bool>("continuous_enabled", false);
    const double continuous_rate_hz = declare_parameter<double>("continuous_rate_hz", 2.5);
    const auto maximum_pair_delta_s = declare_parameter<double>("maximum_pair_delta_s", 0.02);
    max_pair_delta_ns_ = static_cast<int64_t>(maximum_pair_delta_s * 1e9);
    const auto maximum_scan_age_s = declare_parameter<double>("maximum_scan_age_s", 0.75);
    max_scan_age_ns_ = static_cast<int64_t>(maximum_scan_age_s * 1e9);
    rgl::MatchConfig config;
    config.voxel_m = declare_parameter<double>("voxel_m", 0.10);
    config.max_correspondence_m = declare_parameter<double>("max_correspondence_m", 0.35);
    config.overlap_distance_m = declare_parameter<double>("overlap_distance_m", 0.20);
    config.maximum_rmse_m = declare_parameter<double>("maximum_rmse_m", 0.08);
    config.minimum_overlap = declare_parameter<double>("minimum_overlap", 0.55);
    config.minimum_3d_geometry_ratio = declare_parameter<double>("minimum_3d_geometry_ratio", 0.003);
    config.minimum_points = declare_parameter<int>("minimum_points", 80);
    config.maximum_iterations = declare_parameter<int>("maximum_iterations", 35);
    if (map_path.empty() || map_id_.empty() || calibration_id_.empty() || floor_.empty() ||
        odom_epoch_ < 0 || max_pair_delta_ns_ <= 0 || max_scan_age_ns_ <= 0 ||
        continuous_rate_hz < 2.0 || continuous_rate_hz > 5.0 ||
        (continuous_enabled_ && !publish_tf_)) {
      throw std::invalid_argument("approved map/calibration/floor/odom epoch and valid timing are required");
    }
    auto map = rgl::Cloud::Ptr(new rgl::Cloud);
    if (pcl::io::loadPCDFile(map_path, *map) < 0) {
      throw std::invalid_argument("cannot load static PCD map");
    }
    matcher_ = std::make_unique<rgl::GicpMatcher>(map, config);
    minimum_points_ = config.minimum_points;
    // A wider search grid keeps the full-map proposal bounded on dense site maps.
    // GICP still verifies against the map at the configured fine resolution.
    large_map_ = map->size() >= 100000;
    search_voxel_m_ = config.voxel_m * (large_map_ ? 3.0 : 1.0);
    const auto map_points = search_points(map);
    global_search_ = std::make_unique<rgl::GlobalSearch>(map_points, search_voxel_m_, config.max_correspondence_m);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/localization/map", rclcpp::QoS(1).transient_local().reliable());
    sensor_msgs::msg::PointCloud2 map_message;
    pcl::toROSMsg(*map, map_message);
    map_message.header.frame_id = "map";
    map_message.header.stamp = now();
    map_pub_->publish(map_message);
    quality_pub_ = create_publisher<Quality>("/localization/quality", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) { on_odom(std::move(message)); });
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr message) { on_scan(std::move(message)); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/livox/imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr message) {
          const auto &a = message->linear_acceleration;
          const Eigen::Vector3d acceleration(a.x, a.y, a.z);
          if (!acceleration.allFinite() || acceleration.norm() < 0.1) return;
          std::lock_guard<std::mutex> lock(data_mutex_);
          accelerations_.push_back(acceleration);
          while (accelerations_.size() > 50) accelerations_.pop_front();
          imu_stamp_ns_ = stamp_ns(message->header.stamp);
        });
    time_sub_ = create_subscription<robocon_interfaces::msg::TimeAlignmentStatus>(
        "/time_alignment/status", 10,
        [this](robocon_interfaces::msg::TimeAlignmentStatus::SharedPtr message) {
          if (message->sensor != "mid360_ptp_clock") return;
          bool lost = false;
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            // This counter belongs to the MID clock monitor, not FAST-LIO odometry.
            // A changed clock epoch invalidates the map anchor until a fresh
            // localizer/odometry epoch is explicitly started.
            if (message->valid && !clock_reset_baseline_) {
              clock_reset_baseline_ = message->reset_count;
            } else if (clock_reset_baseline_ && message->reset_count != *clock_reset_baseline_) {
              clock_epoch_broken_ = true;
            }
            time_gate_valid_ = message->valid && !clock_epoch_broken_;
            time_gate_received_ = std::chrono::steady_clock::now();
            lost = committed_ && !time_gate_valid_;
            if (lost) committed_ = false;
          }
          if (lost) quality_pub_->publish(rejected_quality("mid_clock_gate_lost", now()));
        });
    epoch_sub_ = create_subscription<std_msgs::msg::UInt64>(
        "/localization/odom_epoch", 10,
        [this](std_msgs::msg::UInt64::SharedPtr message) {
          bool lost = false;
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            // The LIO supervisor must advance this monotonically on every
            // frontend restart. The MID clock reset counter cannot supply it.
            if (message->data != static_cast<uint64_t>(odom_epoch_)) {
              odom_epoch_broken_ = true;
              lost = committed_;
              committed_ = false;
            }
            odom_epoch_seen_ = true;
            odom_epoch_received_ = std::chrono::steady_clock::now();
          }
          if (lost) quality_pub_->publish(rejected_quality("lio_odom_epoch_changed", now()));
        });
    action_server_ = rclcpp_action::create_server<Relocalize>(
        this, "/localization/relocalize",
        [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Relocalize::Goal> goal) {
          return handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandle> handle) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          // Commit and cancel have one linearization point. Once TF is
          // committed, cancellation must not report that it prevented it.
          if (!busy_ || goal_finalizing_ || !active_goal_id_ ||
              *active_goal_id_ != handle->get_goal_id()) {
            return rclcpp_action::CancelResponse::REJECT;
          }
          cancel_requested_ = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandle> goal) {
          if (worker_.joinable()) worker_.join();
          worker_ = std::thread([this, goal] {
            try {
              execute(goal);
            } catch (const std::exception &error) {
              {
                std::lock_guard<std::mutex> lock(data_mutex_);
                committed_ = false;
                active_goal_id_.reset();
                busy_ = false;
              }
              RCLCPP_FATAL(get_logger(), "relocalization worker failed; map TF disabled: %s", error.what());
            } catch (...) {
              {
                std::lock_guard<std::mutex> lock(data_mutex_);
                committed_ = false;
                active_goal_id_.reset();
                busy_ = false;
              }
              RCLCPP_FATAL(get_logger(), "relocalization worker failed with unknown error; map TF disabled");
            }
          });
        });
    tf_timer_ = create_wall_timer(200ms, [this] { republish_committed_tf(); });
    if (continuous_enabled_) {
      const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / continuous_rate_hz));
      continuous_timer_ = create_wall_timer(period, [this] { refresh_localization(); });
    }
    RCLCPP_INFO(get_logger(), "Map loaded: CPU 3D-BBS global search and GICP tracking ready");
  }

  ~GlobalLocalizer() override {
    cancel_requested_ = true;
    if (worker_.joinable()) worker_.join();
  }

 private:
  std::vector<Eigen::Vector3d> search_points(
      const rgl::Cloud::ConstPtr &cloud, float max_range = std::numeric_limits<float>::infinity()) const {
    rgl::Cloud filtered;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(search_voxel_m_, search_voxel_m_, search_voxel_m_);
    filter.filter(filtered);
    std::vector<Eigen::Vector3d> points;
    points.reserve(filtered.size());
    for (const auto &p : filtered) {
      if (pcl::isFinite(p) && p.getVector3fMap().squaredNorm() <= max_range * max_range)
        points.emplace_back(p.x, p.y, p.z);
    }
    return points;
  }

  static int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  static Eigen::Matrix4f matrix(const geometry_msgs::msg::Pose &pose) {
    Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z);
    if (!std::isfinite(q.norm()) || q.norm() < 0.98f || q.norm() > 1.02f) {
      throw std::invalid_argument("invalid pose quaternion");
    }
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    transform(0, 3) = pose.position.x;
    transform(1, 3) = pose.position.y;
    transform(2, 3) = pose.position.z;
    if (!transform.allFinite()) throw std::invalid_argument("nonfinite pose");
    return transform;
  }

  static geometry_msgs::msg::Pose pose(const Eigen::Matrix4f &transform) {
    geometry_msgs::msg::Pose result;
    const Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
    result.position.x = transform(0, 3);
    result.position.y = transform(1, 3);
    result.position.z = transform(2, 3);
    result.orientation.x = q.x();
    result.orientation.y = q.y();
    result.orientation.z = q.z();
    result.orientation.w = q.w();
    return result;
  }

  void on_odom(nav_msgs::msg::Odometry::SharedPtr message) {
    if (message->header.frame_id != "odom" || message->child_frame_id != "mid360_imu") return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto stamp = stamp_ns(message->header.stamp);
    if (stamp <= 0) return;
    if (stamp <= last_odom_stamp_ns_) {
      if (last_odom_stamp_ns_ - stamp > max_pair_delta_ns_) {
        odom_stream_regressed_ = true;
        committed_ = false;
      }
      return;
    }
    last_odom_stamp_ns_ = stamp;
    odoms_.push_back(std::move(message));
    while (odoms_.size() > 40) odoms_.pop_front();
    pair_latest_scan_locked();
  }

  void on_scan(sensor_msgs::msg::PointCloud2::SharedPtr message) {
    if (message->header.frame_id != "mid360_imu") return;
    const auto scan_ns = stamp_ns(message->header.stamp);
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (scan_ns <= 0) return;
    if (scan_ns <= last_scan_stamp_ns_) {
      if (last_scan_stamp_ns_ - scan_ns > max_pair_delta_ns_) {
        scan_stream_regressed_ = true;
        committed_ = false;
      }
      return;
    }
    last_scan_stamp_ns_ = scan_ns;
    latest_scan_ = std::move(message);
    latest_ = {};  // A new scan cannot borrow an older odometry pair.
    pair_latest_scan_locked();
  }

  void pair_latest_scan_locked() {
    if (!latest_scan_) return;
    const auto scan_ns = stamp_ns(latest_scan_->header.stamp);
    nav_msgs::msg::Odometry::SharedPtr best;
    int64_t smallest = max_pair_delta_ns_ + 1;
    for (const auto &odom : odoms_) {
      const int64_t delta = std::llabs(scan_ns - stamp_ns(odom->header.stamp));
      if (delta < smallest) { smallest = delta; best = odom; }
    }
    if (best && smallest <= max_pair_delta_ns_) latest_ = {latest_scan_, best};
  }

  bool time_gate_fresh_locked() const {
    if (clock_epoch_broken_) return false;
    if (!require_live_time_gate_) return true;  // Only for isolated synthetic replay.
    return time_gate_valid_ &&
        std::chrono::steady_clock::now() - time_gate_received_ <
            std::chrono::nanoseconds(max_scan_age_ns_);
  }

  bool odom_epoch_fresh_locked() const {
    if (odom_epoch_broken_) return false;
    if (!require_live_odom_epoch_) return true;  // Isolated synthetic replay only.
    return odom_epoch_seen_ && !odom_epoch_broken_ &&
        std::chrono::steady_clock::now() - odom_epoch_received_ <
            std::chrono::nanoseconds(max_scan_age_ns_);
  }

  bool snapshot_fresh_locked(const Snapshot &snapshot) const {
    if (odom_stream_regressed_ || scan_stream_regressed_ ||
        !snapshot.scan || !snapshot.odom) return false;
    const auto scan_ns = stamp_ns(snapshot.scan->header.stamp);
    const auto odom_ns = stamp_ns(snapshot.odom->header.stamp);
    const auto now_ns = now().nanoseconds();
    const auto scan_age = now_ns - scan_ns;
    const auto odom_age = now_ns - odom_ns;
    return scan_ns > 0 && odom_ns > 0 &&
        scan_age >= -20000000LL && scan_age <= max_scan_age_ns_ &&
        odom_age >= -20000000LL && odom_age <= max_scan_age_ns_ &&
        std::llabs(scan_ns - odom_ns) <= max_pair_delta_ns_;
  }

  bool commit_guard_locked(const Snapshot &snapshot) const {
    return time_gate_fresh_locked() && odom_epoch_fresh_locked() &&
        snapshot_fresh_locked(snapshot);
  }

  bool committed_fresh_locked() const {
    if (!committed_ || !commit_guard_locked(latest_)) return false;
    const auto age = now().nanoseconds() - committed_scan_stamp_ns_;
    return age >= -20000000LL && age <= max_scan_age_ns_;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      const std::shared_ptr<const Relocalize::Goal> &goal) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!committed_fresh_locked()) committed_ = false;
      if (!time_gate_fresh_locked() || !odom_epoch_fresh_locked()) {
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    if (!publish_tf_ || goal->map_id != map_id_ || goal->calibration_id != calibration_id_ ||
        goal->odom_epoch != static_cast<uint64_t>(odom_epoch_) || goal->expected_floor != floor_ ||
        goal->timeout_s <= 0.0f || goal->timeout_s > 30.0f) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      cancel_requested_ = false;
      goal_finalizing_ = false;
      active_goal_id_ = uuid;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void execute(const std::shared_ptr<GoalHandle> &handle) {
    struct BusyReset {
      GlobalLocalizer &node;
      ~BusyReset() {
        std::lock_guard<std::mutex> lock(node.data_mutex_);
        node.active_goal_id_.reset();
        node.busy_ = false;
      }
    };
    BusyReset reset{*this};
    const auto goal = handle->get_goal();
    auto result = std::make_shared<Relocalize::Result>();
    bool committed_this_goal = false;
    result->quality = rejected_quality("not_started", now());
    const auto reject = [this, &handle, &result](const std::string &reason) {
      bool canceled = false;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        canceled = cancel_requested_ || handle->is_canceling();
        goal_finalizing_ = true;
      }
      result->success = false;
      result->tf_committed = false;
      result->message = canceled ? "cancelled before TF commit" : reason;
      result->quality.accepted = false;
      result->quality.tf_committed = false;
      result->quality.rejection_reason = result->message;
      quality_pub_->publish(result->quality);
      if (canceled) {
        // ROS marks the handle CANCELING just after the cancel callback returns.
        for (int i = 0; i < 50 && !handle->is_canceling(); ++i) {
          std::this_thread::sleep_for(1ms);
        }
        if (handle->is_canceling()) handle->canceled(result);
        else handle->abort(result);
      } else {
        handle->abort(result);
      }
    };
    try {
      const auto deadline = std::chrono::steady_clock::now() +
          std::chrono::milliseconds(static_cast<int>(goal->timeout_s * 1000));
      Snapshot snapshot;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        snapshot = latest_;
        if (!commit_guard_locked(snapshot)) {
          throw std::runtime_error("time, odometry epoch, or paired sensor data not fresh");
        }
      }
      result->quality.header.stamp = snapshot.scan->header.stamp;
      auto cloud = rgl::Cloud::Ptr(new rgl::Cloud);
      pcl::fromROSMsg(*snapshot.scan, *cloud);
      std::vector<Eigen::Matrix4f> seeds;
      for (const auto &seed : goal->seeds) {
        if (seed.header.frame_id != "map") throw std::runtime_error("seed is not in map frame");
        seeds.push_back(matrix(seed.pose.pose));
      }
      if (seeds.empty()) {
        Eigen::Vector3d up = Eigen::Vector3d::Zero();
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          const auto imu_age = now().nanoseconds() - imu_stamp_ns_;
          if (accelerations_.empty() || imu_age < -20000000LL || imu_age > max_scan_age_ns_) {
            throw std::runtime_error("global search requires fresh MID-360 IMU gravity");
          }
          for (const auto &a : accelerations_) up += a;
        }
        // Empty seeds request the whole map, independently of the last anchor.
        auto query_points = large_map_ ? search_points(cloud, 20.0f) : search_points(cloud);
        if (large_map_ && query_points.size() < static_cast<std::size_t>(minimum_points_))
          query_points = search_points(cloud);
        if (query_points.size() < static_cast<std::size_t>(minimum_points_)) {
          throw std::runtime_error("insufficient_scan_points");
        }
        const auto refine_time = large_map_ ?
            std::min(300ms, std::chrono::milliseconds(static_cast<int>(goal->timeout_s * 100))) : 0ms;
        const auto coarse = global_search_->search(query_points, up.normalized(), deadline - refine_time);
        const auto query_odom = matrix(snapshot.odom->pose.pose);
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          snapshot = latest_;
          if (!commit_guard_locked(snapshot)) throw std::runtime_error("paired data expired during global search");
        }
        // Coarse search may span several scans. Refine against the current scan
        // after propagating its seed with the independent LIO relative motion.
        const Eigen::Matrix4f relative = query_odom.inverse() * matrix(snapshot.odom->pose.pose);
        for (const auto &candidate : coarse) seeds.push_back(candidate * relative);
        pcl::fromROSMsg(*snapshot.scan, *cloud);
      }
      const auto match = matcher_->match(cloud, seeds, deadline, &cancel_requested_);
      result->quality = quality_from_match(match, snapshot.scan->header.stamp);
      result->message = match.rejection_reason;
      auto feedback = std::make_shared<Relocalize::Feedback>();
      feedback->evaluated_seeds = match.evaluated_seeds;
      feedback->best_rmse_m = std::isfinite(match.rmse_m) ? match.rmse_m : 0.0f;
      feedback->best_overlap_ratio = match.overlap;
      handle->publish_feedback(feedback);
      if (!match.accepted || std::chrono::steady_clock::now() > deadline) {
        reject(match.accepted ? "matching_timeout" : match.rejection_reason);
        return;
      }
      const auto odom_from_imu = matrix(snapshot.odom->pose.pose);
      const auto map_from_odom = match.map_from_scan * odom_from_imu.inverse();
      if (!map_from_odom.allFinite()) throw std::runtime_error("nonfinite map->odom");
      result->pose.header.frame_id = "map";
      result->pose.header.stamp = snapshot.scan->header.stamp;
      result->pose.pose.pose = pose(match.map_from_scan);
      // Conservative placeholder for replay; not a measured field covariance.
      for (const int index : {0, 7, 14, 21, 28, 35}) result->pose.pose.covariance[index] = 1.0;
      result->message = "map->odom committed";
      bool canceled = false;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        canceled = cancel_requested_ || handle->is_canceling();
        if (!canceled) {
          if (!commit_guard_locked(snapshot) ||
              std::chrono::steady_clock::now() > deadline) {
            throw std::runtime_error("time, epoch, paired data or deadline changed before TF commit");
          }
          tf_broadcaster_->sendTransform(tf_message(map_from_odom, snapshot.scan->header.stamp));
          committed_map_from_odom_ = map_from_odom;
          committed_scan_stamp_ns_ = stamp_ns(snapshot.scan->header.stamp);
          committed_ = true;
          // A new action establishes a new anchor. Tracking updates its TF
          // revision without changing this anchor identity.
          result->localization_generation = ++localization_generation_;
          result->tf_revision = ++tf_revision_;
          result->success = true;
          result->tf_committed = true;
          result->quality.tf_committed = true;
          result->quality.localization_generation = result->localization_generation;
          result->quality.tf_revision = result->tf_revision;
          goal_finalizing_ = true;
          committed_this_goal = true;
          quality_pub_->publish(result->quality);
        }
      }
      if (canceled) {
        reject("cancelled before TF commit");
        return;
      }
      handle->succeed(result);
    } catch (const std::exception &error) {
      if (committed_this_goal) {
        // Never report "no TF committed" after the transform was published.
        RCLCPP_ERROR(get_logger(), "TF committed but action completion failed: %s", error.what());
      } else {
        reject(error.what());
      }
    }
  }

  Quality quality_from_match(const rgl::MatchResult &match,
                             const builtin_interfaces::msg::Time &stamp) const {
    Quality quality;
    quality.header.stamp = stamp;
    quality.header.frame_id = "map";
    quality.source = "relocalization";
    quality.floor = floor_;
    quality.map_id = map_id_;
    quality.calibration_id = calibration_id_;
    quality.odom_epoch = odom_epoch_;
    quality.accepted = match.accepted;
    quality.converged = match.converged;
    quality.rmse_m = std::isfinite(match.rmse_m) ? match.rmse_m : 1000.0f;
    quality.overlap_ratio = match.overlap;
    quality.geometry_eigenvalues = match.geometry_eigenvalues;
    quality.rejection_reason = match.rejection_reason;
    return quality;
  }

  Quality rejected_quality(const std::string &reason,
                           const builtin_interfaces::msg::Time &stamp) const {
    rgl::MatchResult rejected;
    rejected.rejection_reason = reason;
    rejected.rmse_m = std::numeric_limits<float>::infinity();
    auto quality = quality_from_match(rejected, stamp);
    quality.accepted = false;
    return quality;
  }

  static geometry_msgs::msg::TransformStamped tf_message(
      const Eigen::Matrix4f &transform, const builtin_interfaces::msg::Time &stamp) {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    message.child_frame_id = "odom";
    const auto value = pose(transform);
    message.transform.translation.x = value.position.x;
    message.transform.translation.y = value.position.y;
    message.transform.translation.z = value.position.z;
    message.transform.rotation = value.orientation;
    return message;
  }

  void republish_committed_tf() {
    if (!publish_tf_) return;
    bool lost = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!committed_) return;
      if (!committed_fresh_locked()) {
        committed_ = false;
        lost = true;
      } else {
        // Validation and publication are atomic with respect to gate/epoch
        // invalidation callbacks. No new wall-clock stamp is fabricated.
        tf_broadcaster_->sendTransform(
            tf_message(committed_map_from_odom_, latest_.scan->header.stamp));
      }
    }
    if (lost) {
      quality_pub_->publish(rejected_quality("tracking_evidence_expired", now()));
      return;
    }
  }

  void refresh_localization() {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) return;
    struct BusyReset {
      std::atomic<bool> &flag;
      ~BusyReset() { flag = false; }
    } reset{busy_};
    Snapshot snapshot;
    Eigen::Matrix4f map_from_odom;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (localization_generation_ == 0 || !commit_guard_locked(latest_)) return;
      // A short mismatch may recover locally. Once the anchor evidence expires,
      // only full-map recovery can establish a new anchor.
      if (now().nanoseconds() - committed_scan_stamp_ns_ > max_scan_age_ns_) return;
      snapshot = latest_;
      map_from_odom = committed_map_from_odom_;
    }
    const auto scan_ns = stamp_ns(snapshot.scan->header.stamp);
    if (scan_ns <= last_continuous_scan_ns_) return;
    last_continuous_scan_ns_ = scan_ns;
    const auto age = now().nanoseconds() - scan_ns;
    if (age < -20000000LL || age > max_scan_age_ns_) return;
    try {
      auto cloud = rgl::Cloud::Ptr(new rgl::Cloud);
      pcl::fromROSMsg(*snapshot.scan, *cloud);
      const auto odom_from_imu = matrix(snapshot.odom->pose.pose);
      const auto deadline = std::chrono::steady_clock::now() + 350ms;
      const auto match = matcher_->match(cloud, {map_from_odom * odom_from_imu}, deadline);
      auto quality = quality_from_match(match, snapshot.scan->header.stamp);
      quality.source = "static_map";
      if (!match.accepted || std::chrono::steady_clock::now() > deadline) {
        quality.accepted = false;
        quality.rejection_reason = match.accepted ? "matching_timeout" : match.rejection_reason;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          committed_ = false;
        }
        quality_pub_->publish(quality);
        return;
      }
      const auto updated = match.map_from_scan * odom_from_imu.inverse();
      if (!updated.allFinite()) return;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!commit_guard_locked(snapshot) ||
            std::chrono::steady_clock::now() > deadline) {
          quality.accepted = false;
          quality.rejection_reason = "evidence_expired_before_continuous_commit";
          quality_pub_->publish(quality);
          return;
        }
        committed_map_from_odom_ = updated;
        committed_scan_stamp_ns_ = scan_ns;
        committed_ = true;
        quality.localization_generation = localization_generation_;
        quality.tf_revision = ++tf_revision_;
        tf_broadcaster_->sendTransform(tf_message(updated, snapshot.scan->header.stamp));
        quality.tf_committed = true;
        quality_pub_->publish(quality);
      }
    } catch (const std::exception &error) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        committed_ = false;
      }
      quality_pub_->publish(rejected_quality(error.what(), now()));
      RCLCPP_ERROR(get_logger(), "continuous map match rejected: %s", error.what());
    }
  }

  std::string map_id_, calibration_id_, floor_;
  int64_t odom_epoch_ = -1, max_pair_delta_ns_ = 0, max_scan_age_ns_ = 0;
  int64_t last_odom_stamp_ns_ = 0, last_scan_stamp_ns_ = 0, committed_scan_stamp_ns_ = 0;
  bool publish_tf_ = false, continuous_enabled_ = false;
  bool require_live_time_gate_ = true, require_live_odom_epoch_ = true;
  bool time_gate_valid_ = false, clock_epoch_broken_ = false;
  bool odom_epoch_seen_ = false, odom_epoch_broken_ = false;
  bool odom_stream_regressed_ = false, scan_stream_regressed_ = false;
  bool goal_finalizing_ = false;
  std::optional<rclcpp_action::GoalUUID> active_goal_id_;
  std::optional<uint32_t> clock_reset_baseline_;
  std::chrono::steady_clock::time_point time_gate_received_{}, odom_epoch_received_{};
  int64_t last_continuous_scan_ns_ = 0;
  std::unique_ptr<rgl::GicpMatcher> matcher_;
  std::unique_ptr<rgl::GlobalSearch> global_search_;
  double search_voxel_m_ = 0.1;
  bool large_map_ = false;
  int minimum_points_ = 80;
  std::mutex data_mutex_;
  std::deque<Eigen::Vector3d> accelerations_;
  int64_t imu_stamp_ns_ = 0;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odoms_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_scan_;
  Snapshot latest_;
  bool committed_ = false;
  Eigen::Matrix4f committed_map_from_odom_ = Eigen::Matrix4f::Identity();
  uint64_t localization_generation_ = 0;
  uint64_t tf_revision_ = 0;
  std::atomic<bool> busy_{false}, cancel_requested_{false};
  std::thread worker_;
  rclcpp::Publisher<Quality>::SharedPtr quality_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<robocon_interfaces::msg::TimeAlignmentStatus>::SharedPtr time_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr epoch_sub_;
  rclcpp_action::Server<Relocalize>::SharedPtr action_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr continuous_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GlobalLocalizer>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("global_localizer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
