#include "static_map.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

class MapBuilder final : public rclcpp::Node {
 public:
  MapBuilder() : Node("map_builder") {
    output_ = declare_parameter<std::string>("output_dir");
    if (output_.empty()) throw std::invalid_argument("output_dir is required");
    const auto origin = declare_parameter<std::vector<double>>("lidar_origin_body");
    if (origin.size() != 3) throw std::invalid_argument("lidar_origin_body must contain FAST-LIO's extrinsic_T");
    lidar_origin_body_ = Eigen::Map<const V3D>(origin.data());
    if (!lidar_origin_body_.allFinite()) throw std::invalid_argument("invalid lidar_origin_body");
    Config config;
    config.key_pose_delta_trans = declare_parameter<double>("key_pose_delta_trans", 0.5);
    config.key_pose_delta_deg = declare_parameter<double>("key_pose_delta_deg", 10.0);
    config.loop_time_tresh = declare_parameter<double>("loop_time_tresh", 10.0);
    config.min_loop_detect_duration = declare_parameter<double>("min_loop_detect_duration", 5.0);
    graph_ = std::make_unique<SimplePGO>(config);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/mapping/map", rclcpp::QoS(1).transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/mapping/path", 1);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/livox/imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr message) {
          if (gravity_ready_) return;
          const auto &a = message->linear_acceleration;
          const Eigen::Vector3d value(a.x, a.y, a.z);
          if (!value.allFinite() || value.norm() < 0.1) return;
          accelerations_.push_back(value);
          while (accelerations_.size() > 50) accelerations_.pop_front();
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/Odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
          if (message->header.frame_id != "odom" || message->child_frame_id != "mid360_imu") return;
          odometry_.push_back(std::move(message));
          while (odometry_.size() > 40) odometry_.pop_front();
          process_pair();
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {
          if (message->header.frame_id != "mid360_imu") return;
          scan_ = std::move(message);
          process_pair();
        });
    save_ = create_service<std_srvs::srv::Trigger>("/mapping/save",
        [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          try {
            auto map = save_map();
            publish_map_and_path(map);
            publish_tf();
            // The replay launcher shuts down on the save response. Let DDS
            // deliver the final reliable cloud before acknowledging the save.
            if (!map_pub_->wait_for_all_acked(std::chrono::seconds(1))) {
              RCLCPP_WARN(get_logger(), "Static map saved; a map subscriber did not acknowledge delivery");
            }
            response->success = true;
            response->message = output_;
          } catch (const std::exception &error) {
            response->success = false;
            response->message = error.what();
          }
        });
    RCLCPP_INFO(get_logger(), "Mapping: 3D-BBS loop candidates, GICP/iSAM2 and static map export via /mapping/save");
  }

  ~MapBuilder() override {
    if (observations_.empty() || saved_observation_count_ == observations_.size()) return;
    try {
      save_map();
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Map save failed: %s", error.what());
    }
  }

 private:
  static int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  void process_pair() {
    if (!scan_ || stamp_ns(scan_->header.stamp) <= last_stamp_) return;
    for (const auto &odom : odometry_) {
      if (stamp_ns(odom->header.stamp) != stamp_ns(scan_->header.stamp)) continue;
      const auto &p = odom->pose.pose;
      const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
      const Eigen::Vector3d translation(p.position.x, p.position.y, p.position.z);
      if (!translation.allFinite() || !std::isfinite(q.norm()) || std::abs(q.norm() - 1.0) > 0.02) {
        throw std::runtime_error("invalid FAST-LIO pose");
      }
      if (!gravity_ready_) {
        if (accelerations_.size() < 20) return;
        Eigen::Vector3d up = Eigen::Vector3d::Zero();
        for (const auto &a : accelerations_) up += a;
        leveling_ = Eigen::Quaterniond::FromTwoVectors(q.normalized() * up, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        gravity_ready_ = true;
      }
      CloudWithPose input;
      input.pose.setTime(scan_->header.stamp.sec, scan_->header.stamp.nanosec);
      input.pose.r = leveling_ * q.normalized().toRotationMatrix();
      input.pose.t = leveling_ * translation;
      const auto stamp = stamp_ns(scan_->header.stamp);
      const bool key_due = graph_->isKeyPose(input.pose);
      const bool observe = key_due || stamp - last_observation_stamp_ >= 500000000LL;
      if (observe) {
        auto raw = CloudType::Ptr(new CloudType);
        pcl::fromROSMsg(*scan_, *raw);
        input.cloud = CloudType::Ptr(new CloudType);
        pcl::UniformSampling<PointType> sample;
        sample.setInputCloud(raw);
        sample.setRadiusSearch(map_resolution_);
        sample.filter(*input.cloud);
        if (input.cloud->size() < 80) return;
      }

      last_stamp_ = stamp;
      latest_stamp_ = scan_->header.stamp;
      const bool closed_loop = collect_loop(false);
      if (key_due) graph_->addKeyPose(input);
      if (key_due || closed_loop) graph_->smoothAndUpdate();
      if (observe) {
        const auto &key = graph_->keyPoses().back();
        observations_.push_back({graph_->keyPoses().size() - 1, key.r_local.transpose() * input.pose.r,
            key.r_local.transpose() * (input.pose.t - key.t_local), input.cloud});
        last_observation_stamp_ = stamp;
      }
      graph_->searchForLoopPairs();
      publish_tf();
      if (key_due || closed_loop) publish_map_and_path(optimized_map());
      return;
    }
  }

  bool collect_loop(bool wait) {
    const auto result = graph_->collectLoop(wait);
    if (!result) return false;
    ++loop_attempts_;
    if (!result->pair) {
      RCLCPP_DEBUG(get_logger(), "Loop rejected: %s", result->reason.c_str());
      ++loop_rejections_[result->reason];
      return false;
    }
    const auto &loop = *result->pair;
    RCLCPP_INFO(get_logger(), "Loop accepted: %zu -> %zu, overlap=%.3f RMSE=%.4f m, total=%zu",
                loop.source_id, loop.target_id, loop.overlap, loop.rmse, graph_->historyPairs().size());
    return true;
  }

  void publish_tf() {
    geometry_msgs::msg::TransformStamped message;
    message.header.frame_id = "map";
    message.child_frame_id = "odom";
    message.header.stamp = latest_stamp_;
    const Eigen::Quaterniond q(graph_->offsetR() * leveling_);
    const auto t = graph_->offsetT();
    message.transform.translation.x = t.x();
    message.transform.translation.y = t.y();
    message.transform.translation.z = t.z();
    message.transform.rotation.x = q.x();
    message.transform.rotation.y = q.y();
    message.transform.rotation.z = q.z();
    message.transform.rotation.w = q.w();
    tf_->sendTransform(message);
  }

  CloudType::Ptr optimized_map() {
    const int count = static_cast<int>(graph_->keyPoses().size());
    return graph_->getSubMap(count - 1, count, map_resolution_);
  }

  void publish_map_and_path(const CloudType::ConstPtr &map) {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*map, message);
    message.header.frame_id = "map";
    message.header.stamp = latest_stamp_;
    map_pub_->publish(message);
    nav_msgs::msg::Path path;
    path.header = message.header;
    for (const auto &key : graph_->keyPoses()) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = key.t_global.x();
      pose.pose.position.y = key.t_global.y();
      pose.pose.position.z = key.t_global.z();
      const Eigen::Quaterniond q(key.r_global);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  CloudType::ConstPtr save_map() {
    if (observations_.empty()) throw std::runtime_error("no keyframes received");
    if (saved_observation_count_ == observations_.size()) return saved_map_;
    if (collect_loop(true)) graph_->smoothAndUpdate();
    graph_->searchForLoopPairs();
    if (collect_loop(true)) graph_->smoothAndUpdate();
    const auto start = std::chrono::steady_clock::now();
    const auto cleaned = buildStaticMap(graph_->keyPoses(), observations_, lidar_origin_body_, map_resolution_);
    if (cleaned.cloud->size() < 80) throw std::runtime_error("static map contains fewer than 80 points");
    const double filter_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const auto root = std::filesystem::path(output_);
    std::filesystem::create_directories(root / "keyframes");
    const auto pending_pcd = root / "map.pcd.tmp";
    if (pcl::io::savePCDFileBinary(pending_pcd.string(), *cleaned.cloud) < 0) {
      throw std::runtime_error("cannot write map.pcd");
    }
    std::ofstream poses(root / "poses.csv");
    poses.exceptions(std::ios::failbit | std::ios::badbit);
    poses << "index,stamp,x,y,z,qx,qy,qz,qw,local_x,local_y,local_z\n" << std::setprecision(16);
    const auto &keys = graph_->keyPoses();
    for (std::size_t i = 0; i < keys.size(); ++i) {
      const auto &k = keys[i];
      const Eigen::Quaterniond q(k.r_global);
      poses << i << ',' << k.time << ',' << k.t_global.x() << ',' << k.t_global.y() << ',' << k.t_global.z()
            << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w()
            << ',' << k.t_local.x() << ',' << k.t_local.y() << ',' << k.t_local.z() << '\n';
      std::ostringstream filename;
      filename << std::setw(6) << std::setfill('0') << i << ".pcd";
      if (pcl::io::savePCDFileBinary((root / "keyframes" / filename.str()).string(), *k.body_cloud) < 0) {
        throw std::runtime_error("cannot write keyframe PCD");
      }
    }
    poses.close();
    const auto pending_metadata = root / "map.yaml.tmp";
    std::ofstream metadata(pending_metadata);
    metadata.exceptions(std::ios::failbit | std::ios::badbit);
    metadata << "frame: map\nup_axis: z\npcd_file: map.pcd\nkeyframes: " << keys.size()
             << "\nloops: " << graph_->historyPairs().size()
             << "\nmethod: FAST-LIO2 + 3D-BBS + GICP + GTSAM iSAM2\n"
             << "loop_attempts: " << loop_attempts_ << "\n"
             << "loop_pairs:";
    if (graph_->historyPairs().empty()) metadata << " []";
    for (const auto &[target, source] : graph_->historyPairs()) metadata << "\n  - [" << target << ", " << source << "]";
    metadata << "\nloop_rejections:";
    if (loop_rejections_.empty()) metadata << " {}";
    for (const auto &[reason, count] : loop_rejections_) metadata << "\n  " << reason << ": " << count;
    metadata << "\nstatic_filter: voxel_ray_visibility\n"
             << "visibility_frames: " << observations_.size() << '\n'
             << "observed_voxels: " << cleaned.occupied_voxels << '\n'
             << "removed_dynamic_voxels: " << cleaned.removed_voxels << '\n'
             << "visibility_input_points: " << cleaned.input_points << '\n'
             << "removed_dynamic_returns: " << cleaned.removed_points << '\n'
             << "map_points: " << cleaned.cloud->size() << '\n'
             << "filter_seconds: " << filter_seconds << '\n';
    metadata.close();
    std::filesystem::rename(pending_pcd, root / "map.pcd");
    std::filesystem::rename(pending_metadata, root / "map.yaml");
    saved_map_ = cleaned.cloud;
    saved_observation_count_ = observations_.size();
    RCLCPP_INFO(get_logger(), "Saved %zu keys, %zu loops, %zu static points (removed %zu voxels in %.2f s) to %s",
                keys.size(), graph_->historyPairs().size(), cleaned.cloud->size(), cleaned.removed_voxels,
                filter_seconds, output_.c_str());
    return saved_map_;
  }

  static constexpr float map_resolution_ = 0.10f;
  std::string output_;
  V3D lidar_origin_body_;
  std::vector<MapObservation> observations_;
  int64_t last_observation_stamp_ = 0;
  std::size_t saved_observation_count_ = 0;
  CloudType::ConstPtr saved_map_;
  std::size_t loop_attempts_ = 0;
  std::map<std::string, std::size_t> loop_rejections_;
  std::unique_ptr<SimplePGO> graph_;
  Eigen::Matrix3d leveling_ = Eigen::Matrix3d::Identity();
  bool gravity_ready_ = false;
  std::deque<Eigen::Vector3d> accelerations_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odometry_;
  sensor_msgs::msg::PointCloud2::SharedPtr scan_;
  int64_t last_stamp_ = 0;
  builtin_interfaces::msg::Time latest_stamp_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapBuilder>());
  rclcpp::shutdown();
  return 0;
}
