// CenterPoint LiDAR people-detector node.
//
// Subscribes to a Velodyne PointCloud2, runs the CUDA-CenterPoint engine
// (filtered to the nuScenes "pedestrian" class), transforms each detection
// into the robot ego frame via TF, and publishes the nearest people as
// bva_msgs/NearbyObstacles on /nearby_people -- a drop-in replacement for the
// ZED-based people detection that fed the same topic.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <bva_msgs/msg/nearby_obstacle.hpp>
#include <bva_msgs/msg/nearby_obstacles.hpp>

#include "centerpoint.h"
#include "postprocess.h"
#include "common.h"

namespace
{
constexpr char kDefaultPlanPath[] =
  "/home/ros/pref_ws/src/Lidar_AI_Solution/CUDA-CenterPoint/model/rpn_centerhead_sim.plan";
constexpr char kDefaultScnPath[] =
  "/home/ros/pref_ws/src/Lidar_AI_Solution/CUDA-CenterPoint/model/centerpoint.scn.onnx";

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// A single CenterPoint pedestrian detection expressed in the ego (base_link) frame.
struct PersonDet
{
  double x;
  double y;
  double cos_theta;
  double sin_theta;
  // Detection confidence; used by the smoother's spawn hysteresis (only
  // detections at/above the spawn threshold may start a new track).
  double score;
};

// A smoothed person emitted by the temporal smoother (carries a stable id).
// vx/vy are the ego-frame (relative-to-robot) finite-difference velocity.
struct SmoothedPerson
{
  int id;
  double x;
  double y;
  double cos_theta;
  double sin_theta;
  double vx;
  double vy;
};

// Ego-frame output row used for range-sorting / capping / marker rendering.
struct OutDet
{
  int id;
  double x;
  double y;
  double cos_theta;
  double sin_theta;
  double vx;
  double vy;
  double range_sq;
};

// Lightweight multi-object temporal smoother (base_link frame, no Eigen).
//
// Positions follow CenterPoint in real time via a per-axis scalar smoothing
// filter; a finite-difference velocity is estimated only to extrapolate (coast)
// a person through the occasional missed frame so detections stop blinking.
class PeopleSmoother
{
public:
  struct Params
  {
    double association_gate_m{0.8};
    double position_measurement_noise{0.15};
    double position_process_noise{0.5};
    double velocity_smoothing_beta{0.6};
    int confirm_hits{2};
    int max_coast_frames{6};
    // Minimum detection score required to spawn a brand-new track. Detections
    // below this (but above the node's maintain pre-filter) may only associate
    // to an existing track, never create one. 0 means "spawn from anything".
    double spawn_score_threshold{0.0};
  };

  void configure(const Params & p) { params_ = p; }

  // Live-update the spawn hysteresis threshold (mirrors the node's live
  // score_threshold) without disturbing existing track state.
  void set_spawn_score_threshold(double v) { params_.spawn_score_threshold = v; }

  // Update with this frame's detections and the time since the previous frame.
  // Returns the confirmed, currently-active people (live or briefly coasting).
  std::vector<SmoothedPerson> update(const std::vector<PersonDet> & dets, double dt)
  {
    // Clamp dt: non-positive or absurd gaps disable motion for this step.
    double mdt = dt;
    if (!std::isfinite(mdt) || mdt < 0.0) {
      mdt = 0.0;
    } else if (mdt > kMaxDt) {
      mdt = kMaxDt;
    }

    // Predict positions forward by velocity, used for association gating only.
    std::vector<double> pred_x(tracks_.size());
    std::vector<double> pred_y(tracks_.size());
    for (size_t i = 0; i < tracks_.size(); ++i) {
      pred_x[i] = tracks_[i].x + tracks_[i].vx * mdt;
      pred_y[i] = tracks_[i].y + tracks_[i].vy * mdt;
    }

    // Greedy nearest-neighbor association under the distance gate.
    const double gate_sq = params_.association_gate_m * params_.association_gate_m;
    struct Pair
    {
      double d2;
      size_t det;
      size_t trk;
    };
    std::vector<Pair> pairs;
    pairs.reserve(dets.size() * tracks_.size());
    for (size_t d = 0; d < dets.size(); ++d) {
      for (size_t t = 0; t < tracks_.size(); ++t) {
        const double dx = dets[d].x - pred_x[t];
        const double dy = dets[d].y - pred_y[t];
        const double d2 = dx * dx + dy * dy;
        if (d2 <= gate_sq) {
          pairs.push_back({d2, d, t});
        }
      }
    }
    std::sort(pairs.begin(), pairs.end(),
      [](const Pair & a, const Pair & b) { return a.d2 < b.d2; });

    std::vector<int> det_to_trk(dets.size(), -1);
    std::vector<bool> trk_matched(tracks_.size(), false);
    std::vector<bool> det_matched(dets.size(), false);
    for (const Pair & pr : pairs) {
      if (det_matched[pr.det] || trk_matched[pr.trk]) {
        continue;
      }
      det_matched[pr.det] = true;
      trk_matched[pr.trk] = true;
      det_to_trk[pr.det] = static_cast<int>(pr.trk);
    }

    const double R = params_.position_measurement_noise;
    const double Q = params_.position_process_noise;
    const double beta = params_.velocity_smoothing_beta;

    // Update matched tracks: smooth toward the detection, refresh velocity.
    for (size_t d = 0; d < dets.size(); ++d) {
      const int ti = det_to_trk[d];
      if (ti < 0) {
        continue;
      }
      Track & tr = tracks_[static_cast<size_t>(ti)];
      const double old_x = tr.x;
      const double old_y = tr.y;

      const double kx = tr.px / (tr.px + R);
      tr.x = tr.x + kx * (dets[d].x - tr.x);
      tr.px = (1.0 - kx) * tr.px + Q;

      const double ky = tr.py / (tr.py + R);
      tr.y = tr.y + ky * (dets[d].y - tr.y);
      tr.py = (1.0 - ky) * tr.py + Q;

      if (mdt > 0.0) {
        const double inst_vx = (tr.x - old_x) / mdt;
        const double inst_vy = (tr.y - old_y) / mdt;
        tr.vx = beta * tr.vx + (1.0 - beta) * inst_vx;
        tr.vy = beta * tr.vy + (1.0 - beta) * inst_vy;
      }

      tr.cos_theta = dets[d].cos_theta;
      tr.sin_theta = dets[d].sin_theta;
      tr.hits += 1;
      tr.misses = 0;
    }

    // Coast unmatched tracks by extrapolating with velocity; drop the expired.
    std::vector<Track> survivors;
    survivors.reserve(tracks_.size());
    for (size_t t = 0; t < tracks_.size(); ++t) {
      Track tr = tracks_[t];
      if (!trk_matched[t]) {
        tr.x += tr.vx * mdt;
        tr.y += tr.vy * mdt;
        tr.px += Q;
        tr.py += Q;
        tr.misses += 1;
        if (tr.misses > params_.max_coast_frames) {
          continue;
        }
      }
      survivors.push_back(tr);
    }
    tracks_.swap(survivors);

    // Spawn tentative tracks from unmatched detections. Hysteresis: only
    // sufficiently-confident detections may start a NEW track; weaker ones were
    // admitted solely to sustain existing tracks via association above.
    for (size_t d = 0; d < dets.size(); ++d) {
      if (det_matched[d]) {
        continue;
      }
      if (dets[d].score < params_.spawn_score_threshold) {
        continue;
      }
      Track tr;
      tr.id = next_id_++;
      tr.x = dets[d].x;
      tr.y = dets[d].y;
      tr.px = R;
      tr.py = R;
      tr.cos_theta = dets[d].cos_theta;
      tr.sin_theta = dets[d].sin_theta;
      tr.hits = 1;
      tr.misses = 0;
      tracks_.push_back(tr);
    }

    // Emit confirmed tracks (live or coasting).
    std::vector<SmoothedPerson> out;
    out.reserve(tracks_.size());
    for (const Track & tr : tracks_) {
      if (tr.hits < params_.confirm_hits) {
        continue;
      }
      out.push_back({tr.id, tr.x, tr.y, tr.cos_theta, tr.sin_theta, tr.vx, tr.vy});
    }
    return out;
  }

private:
  struct Track
  {
    int id{0};
    double x{0.0};
    double y{0.0};
    double px{0.0};
    double py{0.0};
    double vx{0.0};
    double vy{0.0};
    double cos_theta{1.0};
    double sin_theta{0.0};
    int hits{0};
    int misses{0};
  };

  static constexpr double kMaxDt = 0.5;
  Params params_;
  std::vector<Track> tracks_;
  int next_id_{0};
};
}  // namespace

class CenterPointPeopleNode : public rclcpp::Node
{
public:
  CenterPointPeopleNode()
  : rclcpp::Node("centerpoint_people_node")
  {
    points_topic_ = declare_parameter<std::string>("points_topic", "/velodyne_points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/nearby_people");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    plan_path_ = declare_parameter<std::string>("model_plan_path", kDefaultPlanPath);
    scn_path_ = declare_parameter<std::string>("scn_onnx_path", kDefaultScnPath);
    score_threshold_ = declare_parameter<double>("score_threshold", 0.3);
    // Lower hysteresis floor: detections in [maintain, spawn) feed the smoother
    // for association/maintenance only. Clamped to <= score_threshold below.
    maintain_score_threshold_ = declare_parameter<double>("maintain_score_threshold", 0.2);
    max_obstacles_ = declare_parameter<int>("max_obstacles", 16);
    z_offset_ = declare_parameter<double>("z_offset", 0.0);
    intensity_scale_ = declare_parameter<double>("intensity_scale", 1.0);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    marker_topic_ = declare_parameter<std::string>("marker_topic", "centerpoint_people_markers");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);
    bool verbose = declare_parameter<bool>("verbose", false);

    // Temporal smoother (base_link). Smooths live positions and bridges missed
    // frames with a finite-difference velocity so people stop blinking/jittering.
    enable_tracker_ = declare_parameter<bool>("enable_tracker", true);
    PeopleSmoother::Params smoother_params;
    smoother_params.association_gate_m =
      declare_parameter<double>("association_gate_m", 0.8);
    smoother_params.position_measurement_noise =
      declare_parameter<double>("position_measurement_noise", 0.15);
    smoother_params.position_process_noise =
      declare_parameter<double>("position_process_noise", 0.5);
    smoother_params.velocity_smoothing_beta =
      declare_parameter<double>("velocity_smoothing_beta", 0.6);
    smoother_params.confirm_hits = declare_parameter<int>("confirm_hits", 2);
    smoother_params.max_coast_frames = declare_parameter<int>("max_coast_frames", 6);
    // Spawn hysteresis: new tracks require the (higher) score_threshold; the
    // pre-filter below admits anything >= maintain_score_threshold so weak
    // returns can sustain confirmed tracks without creating ghosts.
    smoother_params.spawn_score_threshold = score_threshold_;
    smoother_.configure(smoother_params);

    RCLCPP_INFO(get_logger(), "Loading CenterPoint engine:\n  plan: %s\n  scn : %s",
      plan_path_.c_str(), scn_path_.c_str());

    // Construct and prepare the engine once; reuse across callbacks.
    centerpoint_ = std::make_unique<CenterPoint>(plan_path_, scn_path_, verbose);
    centerpoint_->prepare();

    checkCudaErrors(cudaStreamCreate(&stream_));
    checkCudaErrors(cudaMalloc(
      reinterpret_cast<void **>(&d_points_),
      MAX_POINTS_NUM * params_.feature_num * sizeof(float)));
    host_points_.reserve(MAX_POINTS_NUM * params_.feature_num);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<bva_msgs::msg::NearbyObstacles>(output_topic_, 10);
    if (publish_markers_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    }

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CenterPointPeopleNode::onCloud, this, std::placeholders::_1));

    // Allow a subset of parameters to be retuned at runtime (e.g. from the
    // preference_query web UI via SetParameters). Only the detection knobs
    // below are live; model paths / topics remain startup-only.
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&CenterPointPeopleNode::onSetParameters, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "CenterPoint people detector ready: %s -> %s (target_frame=%s, score>=%.2f, max=%d)",
      points_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
      score_threshold_, max_obstacles_);
  }

  ~CenterPointPeopleNode() override
  {
    if (d_points_ != nullptr) {
      cudaFree(d_points_);
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const size_t num_points = packCloud(*msg);
    if (num_points == 0) {
      publishEmpty(msg->header);
      return;
    }

    // H2D copy and inference.
    checkCudaErrors(cudaMemcpyAsync(
      d_points_, host_points_.data(),
      num_points * params_.feature_num * sizeof(float),
      cudaMemcpyHostToDevice, stream_));
    checkCudaErrors(cudaStreamSynchronize(stream_));

    centerpoint_->doinfer(reinterpret_cast<void *>(d_points_),
      static_cast<unsigned int>(num_points), stream_);
    checkCudaErrors(cudaStreamSynchronize(stream_));

    // Resolve the sensor -> ego transform once for this cloud.
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF %s -> %s unavailable: %s", msg->header.frame_id.c_str(),
        target_frame_.c_str(), ex.what());
      return;
    }

    std::vector<PersonDet> raw_dets;
    raw_dets.reserve(centerpoint_->nms_pred_.size());

    // Pre-filter floor: when the tracker is enabled, admit weaker detections
    // (>= maintain_score_threshold) so they can sustain confirmed tracks via the
    // smoother's association; spawning a new track still requires the higher
    // score_threshold (enforced inside the smoother). With the tracker disabled
    // there is no track maintenance, so fall back to the spawn threshold.
    const double effective_maintain =
      std::min(maintain_score_threshold_, score_threshold_);
    const double prefilter_score =
      enable_tracker_ ? effective_maintain : score_threshold_;

    for (const Bndbox & box : centerpoint_->nms_pred_) {
      if (box.id != Params::pedestrian_class_id) {
        continue;
      }
      if (box.score < prefilter_score) {
        continue;
      }

      // Build the detection pose in the sensor frame (undo the z_offset that was
      // applied only to the inference input), then transform into the ego frame.
      geometry_msgs::msg::PoseStamped in;
      in.header = msg->header;
      in.pose.position.x = box.x;
      in.pose.position.y = box.y;
      in.pose.position.z = box.z - z_offset_;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, box.rt);
      in.pose.orientation = tf2::toMsg(q);

      geometry_msgs::msg::PoseStamped out;
      tf2::doTransform(in, out, tf);

      const double ex = out.pose.position.x;
      const double ey = out.pose.position.y;
      double cos_t;
      double sin_t;
      const double yaw = yawFromQuaternion(out.pose.orientation);
      if (std::isfinite(yaw)) {
        cos_t = std::cos(yaw);
        sin_t = std::sin(yaw);
      } else if (std::hypot(ex, ey) > 1e-3) {
        const double radial = std::atan2(ey, ex);
        cos_t = std::cos(radial);
        sin_t = std::sin(radial);
      } else {
        cos_t = 1.0;
        sin_t = 0.0;
      }

      raw_dets.push_back({ex, ey, cos_t, sin_t, box.score});
    }

    // Temporally smooth the per-frame detections (or pass through unchanged).
    std::vector<SmoothedPerson> persons;
    if (enable_tracker_) {
      const double stamp_sec =
        static_cast<double>(msg->header.stamp.sec) +
        static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
      double dt = 0.0;
      if (have_last_cloud_time_) {
        dt = stamp_sec - last_stamp_sec_;
      }
      last_stamp_sec_ = stamp_sec;
      have_last_cloud_time_ = true;
      persons = smoother_.update(raw_dets, dt);
    } else {
      persons.reserve(raw_dets.size());
      int idx = 0;
      for (const PersonDet & d : raw_dets) {
        persons.push_back({idx++, d.x, d.y, d.cos_theta, d.sin_theta, 0.0, 0.0});
      }
    }

    std::vector<OutDet> dets;
    dets.reserve(persons.size());
    for (const SmoothedPerson & p : persons) {
      dets.push_back({p.id, p.x, p.y, p.cos_theta, p.sin_theta, p.vx, p.vy, p.x * p.x + p.y * p.y});
    }

    std::sort(dets.begin(), dets.end(),
      [](const OutDet & a, const OutDet & b) { return a.range_sq < b.range_sq; });
    const size_t keep =
      std::min<size_t>(dets.size(), static_cast<size_t>(std::max(0, max_obstacles_)));
    dets.resize(keep);

    bva_msgs::msg::NearbyObstacles nearby;
    nearby.header.stamp = msg->header.stamp;
    nearby.header.frame_id = target_frame_;
    nearby.obstacles.reserve(dets.size());
    for (const OutDet & d : dets) {
      bva_msgs::msg::NearbyObstacle o;
      o.x = d.x;
      o.y = d.y;
      o.cos_theta = d.cos_theta;
      o.sin_theta = d.sin_theta;
      o.vx = d.vx;
      o.vy = d.vy;
      nearby.obstacles.push_back(o);
    }
    pub_->publish(nearby);

    if (publish_markers_ && marker_pub_) {
      publishMarkers(nearby.header, dets);
    }
  }

  // Apply live updates for the tunable detection knobs. Runs on the node's
  // executor thread; scalar assignment is safe relative to onCloud under a
  // single-threaded spin. Parameters not handled here are accepted as-is
  // (they were already declared) but have no live effect.
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const rclcpp::Parameter & p : params) {
      const std::string & name = p.get_name();
      if (name == "score_threshold") {
        const double v = p.as_double();
        if (v < 0.0) {
          result.successful = false;
          result.reason = "score_threshold must be >= 0";
          break;
        }
        score_threshold_ = v;
        // Keep the smoother's spawn hysteresis in lockstep with the live
        // (FP-suppressing) score threshold.
        smoother_.set_spawn_score_threshold(score_threshold_);
      } else if (name == "maintain_score_threshold") {
        const double v = p.as_double();
        if (v < 0.0) {
          result.successful = false;
          result.reason = "maintain_score_threshold must be >= 0";
          break;
        }
        maintain_score_threshold_ = v;
      } else if (name == "z_offset") {
        z_offset_ = p.as_double();
      } else if (name == "intensity_scale") {
        const double v = p.as_double();
        if (v < 0.0) {
          result.successful = false;
          result.reason = "intensity_scale must be >= 0";
          break;
        }
        intensity_scale_ = v;
      } else if (name == "max_obstacles") {
        const int v = static_cast<int>(p.as_int());
        if (v < 0) {
          result.successful = false;
          result.reason = "max_obstacles must be >= 0";
          break;
        }
        max_obstacles_ = v;
      }
    }

    if (result.successful) {
      RCLCPP_INFO(get_logger(),
        "Live params updated: score>=%.3f z_offset=%.3f intensity_scale=%.3f max=%d",
        score_threshold_, z_offset_, intensity_scale_, max_obstacles_);
    }
    return result;
  }

  // Packs valid points into host_points_ as [x, y, z(+offset), intensity, time=0].
  // Returns the number of points written (<= MAX_POINTS_NUM).
  size_t packCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    bool has_intensity = false;
    for (const auto & f : msg.fields) {
      if (f.name == "intensity") {
        has_intensity = true;
        break;
      }
    }

    host_points_.clear();

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_i;
    if (has_intensity) {
      it_i = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(msg, "intensity");
    }

    size_t count = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      float intensity = 0.0f;
      if (it_i) {
        intensity = **it_i;
        ++(*it_i);
      }

      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z + static_cast<float>(z_offset_);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (x < params_.min_x_range || x > params_.max_x_range ||
        y < params_.min_y_range || y > params_.max_y_range ||
        z < params_.min_z_range || z > params_.max_z_range)
      {
        continue;
      }

      host_points_.push_back(x);
      host_points_.push_back(y);
      host_points_.push_back(z);
      host_points_.push_back(intensity * static_cast<float>(intensity_scale_));
      host_points_.push_back(0.0f);  // time offset (single sweep)

      if (++count >= MAX_POINTS_NUM) {
        break;
      }
    }
    return count;
  }

  void publishEmpty(const std_msgs::msg::Header & cloud_header)
  {
    bva_msgs::msg::NearbyObstacles nearby;
    nearby.header.stamp = cloud_header.stamp;
    nearby.header.frame_id = target_frame_;
    pub_->publish(nearby);
  }

  void publishMarkers(const std_msgs::msg::Header & header, const std::vector<OutDet> & dets)
  {
    visualization_msgs::msg::MarkerArray arr;

    // Stable per-track marker ids let RViz update people in place (each marker
    // self-expires via its lifetime), so they persist instead of being cleared
    // and recreated every frame.
    for (const OutDet & d : dets) {
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.ns = "centerpoint_people";
      m.id = d.id;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = d.x;
      m.pose.position.y = d.y;
      m.pose.position.z = 0.9;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.6;
      m.scale.y = 0.6;
      m.scale.z = 1.8;
      m.color.r = 1.0f;
      m.color.g = 0.2f;
      m.color.b = 0.2f;
      m.color.a = 0.7f;
      m.lifetime = rclcpp::Duration::from_seconds(0.3);
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

  // Parameters.
  std::string points_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string plan_path_;
  std::string scn_path_;
  std::string marker_topic_;
  double score_threshold_{0.3};
  double maintain_score_threshold_{0.2};
  int max_obstacles_{16};
  double z_offset_{0.0};
  double intensity_scale_{1.0};
  bool publish_markers_{true};
  double tf_timeout_sec_{0.1};

  // Temporal smoother state.
  bool enable_tracker_{true};
  PeopleSmoother smoother_;
  bool have_last_cloud_time_{false};
  double last_stamp_sec_{0.0};

  // Engine + CUDA resources.
  Params params_;
  std::unique_ptr<CenterPoint> centerpoint_;
  cudaStream_t stream_{nullptr};
  float * d_points_{nullptr};
  std::vector<float> host_points_;

  // ROS interfaces.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<bva_msgs::msg::NearbyObstacles>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CenterPointPeopleNode>());
  rclcpp::shutdown();
  return 0;
}
