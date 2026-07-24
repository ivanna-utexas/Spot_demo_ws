// Canonical CUDA-CenterPoint LiDAR pedestrian detection and tracking node.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cuda_runtime.h>
#include <NvInferVersion.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <bva_msgs/msg/nearby_obstacle.hpp>
#include <bva_msgs/msg/nearby_obstacles.hpp>
#include <people_detector/msg/map_person.hpp>
#include <people_detector/msg/map_person_array.hpp>
#include <people_detector/msg/people.hpp>
#include <people_detector/msg/people_array.hpp>

#include "centerpoint.h"
#include "common.h"
#include "postprocess.h"
#include "people_detector/sweep_accumulator.hpp"
#include "people_detector/tracker.hpp"
#include "people_detector/transform_utils.hpp"
#include "people_detector/validation.hpp"

namespace pd = people_detector;

namespace
{

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

pd::RigidTransform toRigid(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & q_msg = transform.transform.rotation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  tf2::Matrix3x3 matrix(q);
  pd::RigidTransform result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result.rotation[row * 3 + col] = matrix[row][col];
    }
  }
  result.translation = {
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z};
  return result;
}

double planarYaw(const pd::RigidTransform & transform)
{
  return std::atan2(transform.rotation[3], transform.rotation[0]);
}

void addDiagnostic(
  diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue pair;
  pair.key = key;
  pair.value = value;
  status.values.push_back(pair);
}

}  // namespace

class CenterPointPeopleNode : public rclcpp::Node
{
public:
  CenterPointPeopleNode()
  : rclcpp::Node("centerpoint_people_node")
  {
    declareAndValidateParameters();
    validateRuntimeAssets();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    accumulator_ = std::make_unique<pd::SweepAccumulator>(
      static_cast<std::size_t>(sweep_count_), MAX_POINTS_NUM, reset_gap_sec_);
    tracker_ = std::make_unique<pd::KalmanTracker>(tracker_params_);

    RCLCPP_INFO(
      get_logger(), "Loading CenterPoint assets:\n  plan: %s\n  scn:  %s",
      plan_path_.c_str(), scn_path_.c_str());
    centerpoint_ = std::make_unique<CenterPoint>(plan_path_, scn_path_, verbose_);
    centerpoint_->prepare();
    const auto cuda_status = cudaStreamCreate(&stream_);
    if (cuda_status != cudaSuccess) {
      throw std::runtime_error(
              std::string("Failed to create CUDA stream: ") + cudaGetErrorString(cuda_status));
    }
    const auto alloc_status = cudaMalloc(
      reinterpret_cast<void **>(&d_points_), MAX_POINTS_NUM * params_.feature_num * sizeof(float));
    if (alloc_status != cudaSuccess) {
      throw std::runtime_error(
              std::string("Failed to allocate CenterPoint input: ") +
              cudaGetErrorString(alloc_status));
    }

    people_pub_ = create_publisher<people_detector::msg::PeopleArray>(people_topic_, 10);
    map_pub_ = create_publisher<people_detector::msg::MapPersonArray>(map_topic_, 10);
    nearby_pub_ = create_publisher<bva_msgs::msg::NearbyObstacles>(nearby_topic_, 10);
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CenterPointPeopleNode::onCloud, this, std::placeholders::_1));
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&CenterPointPeopleNode::onSetParameters, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "CenterPoint is canonical: %s -> {%s, %s, %s, %s}, tracking in %s with %d sweeps",
      points_topic_.c_str(), people_topic_.c_str(), map_topic_.c_str(),
      nearby_topic_.c_str(), marker_topic_.c_str(), tracking_frame_.c_str(), sweep_count_);
  }

  ~CenterPointPeopleNode() override
  {
    centerpoint_.reset();
    if (d_points_ != nullptr) {
      cudaFree(d_points_);
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

private:
  void declareAndValidateParameters()
  {
    points_topic_ = declare_parameter<std::string>("points_topic", "/velodyne_points");
    people_topic_ = declare_parameter<std::string>("people_topic", "/people_detections");
    map_topic_ = declare_parameter<std::string>("map_topic", "/people/map_tracks");
    nearby_topic_ = declare_parameter<std::string>("nearby_topic", "/nearby_people");
    marker_topic_ =
      declare_parameter<std::string>("marker_topic", "/people_detections_markers");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/centerpoint_people/diagnostics");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    plan_path_ = declare_parameter<std::string>("model_plan_path", "");
    scn_path_ = declare_parameter<std::string>("scn_onnx_path", "");
    require_versioned_engine_ =
      declare_parameter<bool>("require_versioned_engine", true);
    score_threshold_ = declare_parameter<double>("score_threshold", 0.5);
    maintain_score_threshold_ =
      declare_parameter<double>("maintain_score_threshold", 0.3);
    sweep_count_ = declare_parameter<int>("sweep_count", 10);
    reset_gap_sec_ = declare_parameter<double>("reset_gap_sec", 1.0);
    max_obstacles_ = declare_parameter<int>("max_obstacles", 16);
    z_offset_ = declare_parameter<double>("z_offset", 0.0);
    intensity_scale_ = declare_parameter<double>("intensity_scale", 1.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    verbose_ = declare_parameter<bool>("verbose", false);

    tracker_params_.association_gate_m =
      declare_parameter<double>("association_gate_m", 1.5);
    tracker_params_.mahalanobis_gate =
      declare_parameter<double>("mahalanobis_gate", 9.21);
    tracker_params_.association_velocity_weight =
      declare_parameter<double>("association_velocity_weight", 0.5);
    tracker_params_.position_measurement_noise =
      declare_parameter<double>("position_measurement_noise", 0.25);
    tracker_params_.velocity_measurement_noise =
      declare_parameter<double>("velocity_measurement_noise", 2.0);
    tracker_params_.position_process_noise =
      declare_parameter<double>("position_process_noise", 1.0);
    tracker_params_.velocity_process_noise =
      declare_parameter<double>("velocity_process_noise", 2.0);
    tracker_params_.confirm_hits = declare_parameter<int>("confirm_hits", 3);
    tracker_params_.max_coast_frames = declare_parameter<int>("max_coast_frames", 6);
    tracker_params_.max_gap_sec = reset_gap_sec_;
    tracker_params_.spawn_score_threshold = score_threshold_;
    tracker_params_.maintain_score_threshold = maintain_score_threshold_;

    pd::validatePipelineParameters({
      tracking_frame_, map_frame_, base_frame_, sweep_count_, max_obstacles_,
      tracker_params_.confirm_hits, tracker_params_.max_coast_frames,
      score_threshold_, maintain_score_threshold_, reset_gap_sec_,
      intensity_scale_, tf_timeout_sec_});
  }

  void validateRuntimeAssets()
  {
    pd::validateModelAssets(
      plan_path_, scn_path_, NV_TENSORRT_MAJOR, require_versioned_engine_);
    int device_count = 0;
    const auto cuda_status = cudaGetDeviceCount(&device_count);
    if (cuda_status != cudaSuccess || device_count < 1) {
      throw std::runtime_error(
              "No CUDA device is visible to centerpoint_people_node. Start the project "
              "container with the NVIDIA runtime on the Jetson.");
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    ++frames_received_;
    const double stamp_sec = stampSeconds(message->header.stamp);
    last_dt_sec_ = 0.0;
    if (have_last_input_stamp_) {
      const double candidate = stamp_sec - last_input_stamp_sec_;
      if (candidate > 0.0 && candidate <= reset_gap_sec_) {
        last_dt_sec_ = candidate;
      }
    }
    last_input_stamp_sec_ = stamp_sec;
    have_last_input_stamp_ = true;

    pd::RigidTransform tracking_from_sensor;
    try {
      tracking_from_sensor = lookupRigid(
        tracking_frame_, message->header.frame_id, message->header.stamp);
    } catch (const tf2::TransformException & error) {
      ++missing_tf_frames_;
      ++dropped_frames_;
      publishAllEmpty(message->header.stamp);
      publishDiagnostics(message->header.stamp, "missing tracking TF", 0, 0, 0.0, callback_start);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "TF %s <- %s unavailable: %s",
        tracking_frame_.c_str(), message->header.frame_id.c_str(), error.what());
      return;
    }

    auto points = unpackCloud(*message);
    input_points_ = points.size();
    if (accumulator_->add(std::move(points), tracking_from_sensor, stamp_sec)) {
      tracker_->reset();
      ++reset_frames_;
    }
    host_points_ = accumulator_->packNewestFrame(
      tracking_from_sensor, stamp_sec, static_cast<float>(z_offset_),
      static_cast<float>(intensity_scale_));
    const std::size_t point_count = host_points_.size() / params_.feature_num;
    if (point_count == 0) {
      const auto tracks = tracker_->update({}, stamp_sec);
      publishOutputs(message->header.stamp, tracks);
      publishDiagnostics(message->header.stamp, "empty point cloud", 0, tracks.size(), 0.0, callback_start);
      return;
    }

    const auto inference_start = std::chrono::steady_clock::now();
    auto cuda_status = cudaMemcpyAsync(
      d_points_, host_points_.data(), host_points_.size() * sizeof(float),
      cudaMemcpyHostToDevice, stream_);
    if (cuda_status != cudaSuccess) {
      throw std::runtime_error(
              std::string("CenterPoint H2D copy failed: ") + cudaGetErrorString(cuda_status));
    }
    centerpoint_->doinfer(d_points_, static_cast<unsigned int>(point_count), stream_);
    cuda_status = cudaStreamSynchronize(stream_);
    if (cuda_status != cudaSuccess) {
      throw std::runtime_error(
              std::string("CenterPoint inference failed: ") + cudaGetErrorString(cuda_status));
    }
    const double inference_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - inference_start).count();

    std::vector<pd::Detection> detections;
    for (const Bndbox & box : centerpoint_->nms_pred_) {
      if (box.id != Params::pedestrian_class_id ||
        box.score < maintain_score_threshold_)
      {
        continue;
      }
      pd::Point4 sensor_point{
        box.x, box.y, box.z - static_cast<float>(z_offset_), 0.0F};
      const auto tracking_point = tracking_from_sensor.apply(sensor_point);
      const auto tracking_velocity =
        pd::rotateVector(tracking_from_sensor, box.vx, box.vy);
      detections.push_back({
        tracking_point.x, tracking_point.y, tracking_point.z,
        tracking_velocity[0], tracking_velocity[1],
        box.l, box.w, box.h, box.rt + planarYaw(tracking_from_sensor), box.score});
    }
    detections_count_ = detections.size();

    const std::size_t resets_before = tracker_->resetCount();
    const auto tracks = tracker_->update(detections, stamp_sec);
    if (tracker_->resetCount() != resets_before) {
      ++reset_frames_;
      accumulator_->clear();
      accumulator_->add(unpackCloud(*message), tracking_from_sensor, stamp_sec);
    }
    publishOutputs(message->header.stamp, tracks);
    publishDiagnostics(message->header.stamp, "OK", detections.size(), tracks.size(), inference_ms, callback_start);
  }

  std::vector<pd::Point4> unpackCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    bool has_intensity = false;
    for (const auto & field : cloud.fields) {
      has_intensity = has_intensity || field.name == "intensity";
    }
    std::vector<pd::Point4> points;
    points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> intensity;
      if (has_intensity) {
        intensity =
          std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(cloud, "intensity");
      }
      for (; x != x.end(); ++x, ++y, ++z) {
        const float value = intensity ? **intensity : 0.0F;
        if (intensity) {
          ++(*intensity);
        }
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z) &&
          std::isfinite(value))
        {
          points.push_back({*x, *y, *z, value});
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Invalid PointCloud2 layout: %s", error.what());
    }
    return points;
  }

  pd::RigidTransform lookupRigid(
    const std::string & target, const std::string & source,
    const builtin_interfaces::msg::Time & stamp, double timeout_sec = -1.0) const
  {
    const double timeout = timeout_sec < 0.0 ? tf_timeout_sec_ : timeout_sec;
    return toRigid(tf_buffer_->lookupTransform(
        target, source, stamp, rclcpp::Duration::from_seconds(timeout)));
  }

  void publishOutputs(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<pd::TrackState> & tracks)
  {
    publishPeople(stamp, tracks);
    publishMap(stamp, tracks);
    publishNearby(stamp, tracks);
    if (publish_markers_) {
      publishMarkers(stamp, tracks);
    }
  }

  void publishPeople(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<pd::TrackState> & tracks)
  {
    people_detector::msg::PeopleArray output;
    output.header.stamp = stamp;
    output.header.frame_id = tracking_frame_;
    for (const auto & track : tracks) {
      people_detector::msg::People person;
      person.id = track.id;
      person.source = "centerpoint";
      person.label = "pedestrian";
      person.confidence = static_cast<float>(track.score);
      person.is_human = true;
      person.position.x = track.x;
      person.position.y = track.y;
      person.position.z = track.z;
      person.velocity.x = track.vx;
      person.velocity.y = track.vy;
      person.size.x = track.length;
      person.size.y = track.width;
      person.size.z = track.height;
      output.people.push_back(person);
    }
    people_pub_->publish(output);
  }

  void publishMap(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<pd::TrackState> & tracks)
  {
    people_detector::msg::MapPersonArray output;
    output.header.stamp = stamp;
    output.header.frame_id = map_frame_;
    pd::RigidTransform map_from_tracking;
    try {
      map_from_tracking = lookupRigid(map_frame_, tracking_frame_, stamp, 0.0);
    } catch (const tf2::TransformException &) {
      ++missing_tf_frames_;
      map_pub_->publish(output);
      return;
    }
    for (const auto & track : tracks) {
      people_detector::msg::MapPerson person;
      const auto position = map_from_tracking.apply(pd::Point4{
          static_cast<float>(track.x), static_cast<float>(track.y),
          static_cast<float>(track.z), 0.0F});
      const auto velocity = pd::rotateVector(map_from_tracking, track.vx, track.vy);
      person.id = track.id;
      person.source = "centerpoint";
      person.confidence = static_cast<float>(track.score);
      person.sample_time_sec = stampSeconds(stamp);
      person.track_age_sec = track.age_sec;
      person.dt_sec = last_dt_sec_;
      person.position.x = position.x;
      person.position.y = position.y;
      person.position.z = position.z;
      person.velocity.x = velocity[0];
      person.velocity.y = velocity[1];
      person.velocity.z = velocity[2];
      person.size.x = track.length;
      person.size.y = track.width;
      person.size.z = track.height;
      const std::array<double, 9> position_covariance{
        track.covariance[0], track.covariance[1], 0.0,
        track.covariance[4], track.covariance[5], 0.0,
        0.0, 0.0, 0.25};
      const std::array<double, 9> velocity_covariance{
        track.covariance[10], track.covariance[11], 0.0,
        track.covariance[14], track.covariance[15], 0.0,
        0.0, 0.0, 1.0};
      person.position_covariance = pd::rotateCovariance(
        map_from_tracking, position_covariance);
      person.velocity_covariance = pd::rotateCovariance(
        map_from_tracking, velocity_covariance);
      output.people.push_back(person);
    }
    map_pub_->publish(output);
  }

  void publishNearby(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<pd::TrackState> & tracks)
  {
    bva_msgs::msg::NearbyObstacles output;
    output.header.stamp = stamp;
    output.header.frame_id = base_frame_;
    pd::RigidTransform base_from_tracking;
    try {
      base_from_tracking = lookupRigid(base_frame_, tracking_frame_, stamp, 0.0);
    } catch (const tf2::TransformException &) {
      ++missing_tf_frames_;
      nearby_pub_->publish(output);
      return;
    }
    struct Candidate
    {
      double range_squared;
      bva_msgs::msg::NearbyObstacle obstacle;
    };
    std::vector<Candidate> candidates;
    for (const auto & track : tracks) {
      const auto position = base_from_tracking.apply(pd::Point4{
          static_cast<float>(track.x), static_cast<float>(track.y),
          static_cast<float>(track.z), 0.0F});
      const auto velocity = pd::rotateVector(base_from_tracking, track.vx, track.vy);
      const double yaw = track.yaw + planarYaw(base_from_tracking);
      bva_msgs::msg::NearbyObstacle obstacle;
      obstacle.x = position.x;
      obstacle.y = position.y;
      obstacle.cos_theta = std::cos(yaw);
      obstacle.sin_theta = std::sin(yaw);
      obstacle.vx = velocity[0];
      obstacle.vy = velocity[1];
      candidates.push_back({
        static_cast<double>(position.x) * position.x +
        static_cast<double>(position.y) * position.y, obstacle});
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & left, const Candidate & right) {
        return left.range_squared < right.range_squared;
      });
    const auto keep = std::min<std::size_t>(
      candidates.size(), static_cast<std::size_t>(max_obstacles_));
    for (std::size_t index = 0; index < keep; ++index) {
      output.obstacles.push_back(candidates[index].obstacle);
    }
    nearby_pub_->publish(output);
  }

  void publishMarkers(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<pd::TrackState> & tracks)
  {
    visualization_msgs::msg::MarkerArray output;
    for (const auto & track : tracks) {
      visualization_msgs::msg::Marker box;
      box.header.stamp = stamp;
      box.header.frame_id = tracking_frame_;
      box.ns = "centerpoint_boxes";
      box.id = track.id;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = track.x;
      box.pose.position.y = track.y;
      box.pose.position.z = track.z;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, track.yaw);
      box.pose.orientation.x = orientation.x();
      box.pose.orientation.y = orientation.y();
      box.pose.orientation.z = orientation.z();
      box.pose.orientation.w = orientation.w();
      box.scale.x = std::max(0.05, track.length);
      box.scale.y = std::max(0.05, track.width);
      box.scale.z = std::max(0.05, track.height);
      box.color.r = 0.95F;
      box.color.g = 0.2F;
      box.color.b = 0.15F;
      box.color.a = 0.55F;
      box.lifetime = rclcpp::Duration::from_seconds(0.35);
      output.markers.push_back(box);

      auto velocity = box;
      velocity.ns = "centerpoint_velocity";
      velocity.id = track.id;
      velocity.type = visualization_msgs::msg::Marker::ARROW;
      velocity.pose.orientation = geometry_msgs::msg::Quaternion();
      velocity.pose.orientation.w = 1.0;
      velocity.points.resize(2);
      velocity.points[0] = box.pose.position;
      velocity.points[1] = box.pose.position;
      velocity.points[1].x += track.vx;
      velocity.points[1].y += track.vy;
      velocity.scale.x = 0.06;
      velocity.scale.y = 0.12;
      velocity.scale.z = 0.12;
      velocity.color.r = 0.1F;
      velocity.color.g = 0.8F;
      velocity.color.b = 1.0F;
      velocity.color.a = 0.9F;
      output.markers.push_back(velocity);

      auto text = box;
      text.ns = "centerpoint_ids";
      text.id = track.id;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.orientation.w = 1.0;
      text.pose.position.z += track.height * 0.5 + 0.25;
      text.scale.z = 0.25;
      text.color.r = text.color.g = text.color.b = text.color.a = 1.0F;
      std::ostringstream label;
      label << "person " << track.id << " " << std::lround(track.score * 100.0) << "%";
      text.text = label.str();
      output.markers.push_back(text);
    }
    marker_pub_->publish(output);
  }

  void publishAllEmpty(const builtin_interfaces::msg::Time & stamp)
  {
    publishPeople(stamp, {});
    people_detector::msg::MapPersonArray map;
    map.header.stamp = stamp;
    map.header.frame_id = map_frame_;
    map_pub_->publish(map);
    bva_msgs::msg::NearbyObstacles nearby;
    nearby.header.stamp = stamp;
    nearby.header.frame_id = base_frame_;
    nearby_pub_->publish(nearby);
    if (publish_markers_) {
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker clear;
      clear.header.stamp = stamp;
      clear.header.frame_id = tracking_frame_;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear);
      marker_pub_->publish(markers);
    }
  }

  void publishDiagnostics(
    const builtin_interfaces::msg::Time & stamp, const std::string & message,
    std::size_t detections, std::size_t tracks, double inference_ms,
    const std::chrono::steady_clock::time_point & callback_start)
  {
    const double callback_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "centerpoint_people/pipeline";
    status.hardware_id = "cuda_centerpoint";
    status.level = message == "OK" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = message;
    addDiagnostic(status, "sweep_count", std::to_string(accumulator_->size()));
    addDiagnostic(status, "input_points", std::to_string(input_points_));
    addDiagnostic(status, "packed_points", std::to_string(host_points_.size() / 5));
    addDiagnostic(status, "detections", std::to_string(detections));
    addDiagnostic(status, "confirmed_tracks", std::to_string(tracks));
    addDiagnostic(status, "inference_ms", std::to_string(inference_ms));
    addDiagnostic(status, "callback_ms", std::to_string(callback_ms));
    addDiagnostic(status, "missing_tf_frames", std::to_string(missing_tf_frames_));
    addDiagnostic(status, "dropped_frames", std::to_string(dropped_frames_));
    addDiagnostic(status, "reset_frames", std::to_string(reset_frames_));
    addDiagnostic(status, "frames_received", std::to_string(frames_received_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "score_threshold") {
        const double value = parameter.as_double();
        if (value < maintain_score_threshold_) {
          result.successful = false;
          result.reason = "score_threshold must be >= maintain_score_threshold";
          return result;
        }
        score_threshold_ = value;
        tracker_->setSpawnThreshold(value);
      } else if (parameter.get_name() == "maintain_score_threshold" ||
        parameter.get_name() == "sweep_count" ||
        parameter.get_name() == "tracking_frame")
      {
        result.successful = false;
        result.reason = parameter.get_name() + " requires a node restart";
        return result;
      } else if (parameter.get_name() == "max_obstacles") {
        const int value = static_cast<int>(parameter.as_int());
        if (value < 0) {
          result.successful = false;
          result.reason = "max_obstacles must be >= 0";
          return result;
        }
        max_obstacles_ = value;
      }
    }
    return result;
  }

  std::string points_topic_;
  std::string people_topic_;
  std::string map_topic_;
  std::string nearby_topic_;
  std::string marker_topic_;
  std::string diagnostics_topic_;
  std::string tracking_frame_;
  std::string map_frame_;
  std::string base_frame_;
  std::string plan_path_;
  std::string scn_path_;
  bool require_versioned_engine_{true};
  bool publish_markers_{true};
  bool verbose_{false};
  double score_threshold_{0.5};
  double maintain_score_threshold_{0.3};
  double reset_gap_sec_{1.0};
  double z_offset_{0.0};
  double intensity_scale_{1.0};
  double tf_timeout_sec_{0.05};
  double last_dt_sec_{0.0};
  double last_input_stamp_sec_{0.0};
  bool have_last_input_stamp_{false};
  int sweep_count_{10};
  int max_obstacles_{16};
  pd::TrackerParams tracker_params_;
  Params params_;

  std::unique_ptr<pd::SweepAccumulator> accumulator_;
  std::unique_ptr<pd::KalmanTracker> tracker_;
  std::unique_ptr<CenterPoint> centerpoint_;
  cudaStream_t stream_{nullptr};
  float * d_points_{nullptr};
  std::vector<float> host_points_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<people_detector::msg::PeopleArray>::SharedPtr people_pub_;
  rclcpp::Publisher<people_detector::msg::MapPersonArray>::SharedPtr map_pub_;
  rclcpp::Publisher<bva_msgs::msg::NearbyObstacles>::SharedPtr nearby_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

  std::size_t frames_received_{0};
  std::size_t input_points_{0};
  std::size_t detections_count_{0};
  std::size_t missing_tf_frames_{0};
  std::size_t dropped_frames_{0};
  std::size_t reset_frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CenterPointPeopleNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("centerpoint_people_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
