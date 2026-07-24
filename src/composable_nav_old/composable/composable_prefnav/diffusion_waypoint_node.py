from __future__ import annotations

import copy
import math
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import hydra
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from people_detector.msg import People as Track
from people_detector.msg import PeopleArray as TrackArray
from hydra.core.global_hydra import GlobalHydra
from nav_msgs.msg import Odometry, Path as PathMsg
from omegaconf import OmegaConf
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float32MultiArray
from torch.func import stack_module_state
from vlm_policy_msgs.msg import PedAction, PedActionArray
from visualization_msgs.msg import Marker, MarkerArray

from prefnav.datasets.environment_2d.obstacles import Human, RectangleCornerAvoid, RectangleCornerPrefer
from prefnav.misc.common import find_first_waypoint_within_radius, load_data
from prefnav.misc.coordinate_transforms import coordinate_transform, coordinate_transform_to_global
from prefnav.misc.inference import construct_context_state_conds
from prefnav.misc.normalizers import PathNormalizer
from prefnav.misc.process_data import (
    construct_normalized_dynamic_obstacle_from_obj,
    construct_normalized_prefer_region,
    construct_normalized_static_obstacle_from_obj,
)
from prefnav.models.diffusion import make_timesteps
from prefnav.robot_deployment.diffusion_utils import (
    fast_interpolate_path,
    get_guided_diffusion_path_vmap,
    load_diffusion_models,
    repeat_context,
    stack_dictionaries_list,
)
from prefnav.train.dataloader_base import ProcessObsHelper


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class DiffusionWaypointNode(Node):
    def __init__(self) -> None:
        super().__init__("prefnav_diffusion_waypoint_node")

        self._declare_parameters()

        self.device = self.get_parameter("device").value
        self.prefnav_config_name = self.get_parameter("prefnav_config_name").value
        self.scenario_file = self.get_parameter("scenario_file").value
        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.action_timeout_sec = float(self.get_parameter("action_timeout_sec").value)
        self.max_past_odom_samples = int(self.get_parameter("max_past_odom_samples").value)
        self.default_human_radius = float(self.get_parameter("default_human_radius").value)
        self.overtake_model_key = self.get_parameter("overtake_model_key").value

        self._compose_runtime_config()
        self._load_scenario()
        self._initialize_diffusion()

        self.latest_tracks: Optional[TrackArray] = None
        self.latest_odom_frame_id: str = "odom"
        self.latest_actions: Dict[int, PedAction] = {}
        self.latest_actions_stamp = self.get_clock().now()

        self.start_pose: Optional[List[float]] = None
        self.diffusion_path = None
        self.curr_diff_state_idx = 0
        self.past_odoms: List[List[float]] = []
        self.past_odoms_lock = threading.Lock()

        self.path_pub = self.create_publisher(PathMsg, self.get_parameter("waypoints_topic").value, _reliable_qos())
        self.legacy_path_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter("legacy_diffusion_path_topic").value,
            _reliable_qos(),
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("markers_topic").value,
            _reliable_qos(),
        )

        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_callback, _reliable_qos())
        self.create_subscription(TrackArray, self.get_parameter("tracks_topic").value, self._tracks_callback, _reliable_qos())
        self.create_subscription(
            PedActionArray,
            self.get_parameter("ped_actions_topic").value,
            self._ped_actions_callback,
            _reliable_qos(),
        )

        replan_period = float(self.cfg.diffusion_replan_dt)
        self.create_timer(replan_period, self._replan)

        self.get_logger().info(
            "PrefNav diffusion waypoint node ready. "
            f"odom={self.get_parameter('odom_topic').value}, "
            f"tracks={self.get_parameter('tracks_topic').value}, "
            f"ped_actions={self.get_parameter('ped_actions_topic').value}"
        )

    def _declare_parameters(self) -> None:
        prefnav_share = Path(get_package_share_directory("prefnav"))
        default_scenario = prefnav_share / "robot_deployment" / "task_scenarios" / "robot_exp" / "ahg_test_open.yaml"

        self.declare_parameter("prefnav_config_name", "exps/robot_exp/deploy")
        self.declare_parameter("scenario_file", str(default_scenario))
        self.declare_parameter("device", "cuda")
        self.declare_parameter("compile", False)
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("tracks_topic", "/people_detections")
        self.declare_parameter("ped_actions_topic", "social_nav/ped_actions")
        self.declare_parameter("waypoints_topic", "prefnav/diffusion_waypoints")
        self.declare_parameter("legacy_diffusion_path_topic", "prefnav/diffusion_path")
        self.declare_parameter("markers_topic", "prefnav/diffusion_waypoint_markers")
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("action_timeout_sec", 2.0)
        self.declare_parameter("max_past_odom_samples", 250)
        self.declare_parameter("default_human_radius", 0.5)
        self.declare_parameter("overtake_model_key", "overtake_right")
        self.declare_parameter("pretrain_dynamic_checkpoint", "")
        self.declare_parameter("pretrain_static_checkpoint", "")
        self.declare_parameter("follow_checkpoint", "")
        self.declare_parameter("overtake_left_checkpoint", "")
        self.declare_parameter("overtake_right_checkpoint", "")
        self.declare_parameter("yield_checkpoint", "")
        self.declare_parameter("prefer_region_checkpoint", "")

    def _compose_runtime_config(self) -> None:
        conf_dir = Path(get_package_share_directory("prefnav")) / "conf"
        overrides = [
            f"scenario_fn={self.scenario_file}",
            f"compile={str(self.get_parameter('compile').value).lower()}",
        ]

        checkpoint_params = {
            "pretrain_dynamic": self.get_parameter("pretrain_dynamic_checkpoint").value,
            "pretrain_static": self.get_parameter("pretrain_static_checkpoint").value,
            "follow": self.get_parameter("follow_checkpoint").value,
            "overtake_left": self.get_parameter("overtake_left_checkpoint").value,
            "overtake_right": self.get_parameter("overtake_right_checkpoint").value,
            "yield": self.get_parameter("yield_checkpoint").value,
            "prefer_region": self.get_parameter("prefer_region_checkpoint").value,
        }
        for model_key, checkpoint in checkpoint_params.items():
            quoted_checkpoint = checkpoint if checkpoint else "''"
            overrides.append(f"inference.eval_models.{model_key}.checkpoint={quoted_checkpoint}")

        GlobalHydra.instance().clear()
        with hydra.initialize_config_dir(version_base=None, config_dir=str(conf_dir)):
            self.cfg = hydra.compose(config_name=self.prefnav_config_name, overrides=overrides)

        missing = []
        for model_key, model_cfg in self.cfg.inference.eval_models.items():
            checkpoint_path = Path(str(model_cfg.checkpoint)).expanduser()
            if not checkpoint_path.is_file():
                missing.append(f"{model_key}: {checkpoint_path}")
        if missing:
            missing_text = "; ".join(missing)
            raise FileNotFoundError(
                "Missing diffusion checkpoints. Set the *_checkpoint ROS parameters. "
                f"Current values: {missing_text}"
            )

    def _load_scenario(self) -> None:
        scenario_data = OmegaConf.create(load_data(self.scenario_file))
        self.goal_pos = np.asarray(getattr(scenario_data, "goal_pos"), dtype=np.float32)
        self.diffusion_goal_pos = np.asarray(
            getattr(scenario_data, "diffusion_goal_pos", getattr(scenario_data, "goal_pos")),
            dtype=np.float32,
        )
        self.goal_radius = float(getattr(scenario_data, "goal_radius"))
        self.guidance_weight = float(getattr(scenario_data, "guidance_weight"))
        self.mult_x = float(getattr(scenario_data, "mult_x"))
        self.mult_y = float(getattr(scenario_data, "mult_y"))
        self.total_batch_size = int(getattr(scenario_data, "total_batch_size"))

        self.static_obs = []
        self.terrain_obs = []
        for entity in getattr(scenario_data, "entities", []):
            entity_type = entity.get("type")
            if entity_type == "RectangleCornerAvoid":
                self.static_obs.append(
                    RectangleCornerAvoid(entity["top"], entity["bottom"], entity["left"], entity["right"])
                )
            elif entity_type == "RectangleCornerPrefer":
                self.terrain_obs.append(
                    RectangleCornerPrefer(entity["top"], entity["bottom"], entity["left"], entity["right"])
                )

    def _initialize_diffusion(self) -> None:
        self.grid_size = int(self.cfg.grid_size)
        self.max_planning_time = int(self.cfg.max_planning_time)
        self.diffusion_planning_dt = float(self.cfg.diffusion_planning_dt)
        self.odometry_dt = float(self.cfg.odometry_dt)
        self.max_obj_traj_len = int(self.cfg.max_obj_traj_len)
        self.max_padded_obj_num = int(self.cfg.max_padded_obj_num)
        self.max_padded_terrain_num = int(self.cfg.max_padded_terrain_num)
        self.pos_offset = np.asarray(self.cfg.pos_offset, dtype=np.float32)
        self.replan_add_noise_step = int(self.cfg.replan_add_noise_step)
        self.compile = bool(self.cfg.compile)

        self.diffusion_models = load_diffusion_models(self.cfg, self.device)
        self.path_normalizer = PathNormalizer(self.grid_size)
        self.process_obs_helper = ProcessObsHelper(
            self.max_obj_traj_len,
            self.max_padded_obj_num,
            self.max_padded_terrain_num,
        )
        goal_pos_offset = self.diffusion_goal_pos + self.pos_offset[:2]
        self.goal_pos_normalized = self.path_normalizer.normalize(goal_pos_offset)
        self.pos_offset_normalized = self.path_normalizer.normalize(self.pos_offset)[:2]

        self.anchor_model = self.diffusion_models["pretrain_dynamic"]

    def _tracks_callback(self, msg: TrackArray) -> None:
        self.latest_tracks = msg

    def _ped_actions_callback(self, msg: PedActionArray) -> None:
        self.latest_actions = {int(action.track_id): action for action in msg.actions}
        self.latest_actions_stamp = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        yaw = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_euler(
            "xyz",
            degrees=False,
        )[2]

        curr_pose = [position.x, position.y, yaw]
        self.latest_odom_frame_id = msg.header.frame_id or self.latest_odom_frame_id
        if self.start_pose is None:
            self.start_pose = curr_pose
            self.get_logger().info(f"Captured start pose: {self.start_pose}")

        x, y, theta = coordinate_transform(self.start_pose, curr_pose)
        x += float(self.pos_offset[0])
        with self.past_odoms_lock:
            self.past_odoms.append([x, y, theta])
            if len(self.past_odoms) > self.max_past_odom_samples:
                self.past_odoms = self.past_odoms[-self.max_past_odom_samples :]

    def _build_dynamic_obstacles(self) -> Tuple[List[Human], List[str]]:
        if self.latest_tracks is None or self.start_pose is None:
            return [], []

        dynamic_obs: List[Human] = []
        model_keys: List[str] = []
        for track in sorted(self.latest_tracks.people, key=lambda item: item.id):
            local_pos, local_vel = self._track_to_local_state(track)
            future_positions = np.stack(
                [local_pos + local_vel * self.diffusion_planning_dt * step for step in range(self.max_planning_time)],
                axis=0,
            ).astype(np.float32)
            dynamic_obs.append(
                Human(
                    future_positions,
                    name=f"track_{track.id}",
                    radius=self.default_human_radius,
                    dt_interval=self.diffusion_planning_dt,
                )
            )
            model_keys.append(self._action_to_model_key(track.id))
        return dynamic_obs, model_keys

    def _track_to_local_state(self, track: Track) -> Tuple[np.ndarray, np.ndarray]:
        local_pose = coordinate_transform(
            self.start_pose, [track.position.x, track.position.y, 0.0]
        )
        theta = -float(self.start_pose[2])
        rotation = np.array(
            [
                [math.cos(theta), -math.sin(theta)],
                [math.sin(theta), math.cos(theta)],
            ],
            dtype=np.float32,
        )
        global_vel = np.array([track.velocity.x, track.velocity.y], dtype=np.float32)
        local_vel = rotation @ global_vel
        return np.array(local_pose[:2], dtype=np.float32), local_vel

    def _action_to_model_key(self, track_id: int) -> str:
        if (self.get_clock().now() - self.latest_actions_stamp) > Duration(seconds=self.action_timeout_sec):
            return "pretrain_dynamic"

        action = self.latest_actions.get(track_id)
        if action is None:
            return "pretrain_dynamic"
        if action.action == PedAction.ACTION_FOLLOW:
            return "follow"
        if action.action == PedAction.ACTION_YIELD_TO:
            return "yield"
        if action.action == PedAction.ACTION_OVERTAKE:
            return self.overtake_model_key
        return "pretrain_dynamic"

    def _build_tasks(self, dynamic_model_keys: List[str]) -> List[dict]:
        tasks = []
        for idx in range(len(self.static_obs)):
            tasks.append(
                {
                    "context_field": "static_obs_encoder",
                    "model_key": "pretrain_static",
                    "weight": 1,
                    "obs_idx": [idx],
                }
            )
        for idx in range(len(self.terrain_obs)):
            tasks.append(
                {
                    "context_field": "terrain_encoder",
                    "model_key": "prefer_region",
                    "weight": 1,
                    "obs_idx": [idx],
                }
            )
        for idx, model_key in enumerate(dynamic_model_keys):
            tasks.append(
                {
                    "context_field": "dynamic_obs_encoder",
                    "model_key": model_key,
                    "weight": 1,
                    "obs_idx": [idx],
                }
            )
        return tasks

    def _context_constructor(self, dynamic_obs, static_obs, terrain_obs):
        normalized_dynamic = construct_normalized_dynamic_obstacle_from_obj(
            dynamic_obs,
            self.grid_size,
            self.max_planning_time,
            self.diffusion_planning_dt,
            mult_x=self.mult_x,
            mult_y=self.mult_y,
            offset_x=float(self.pos_offset[0]),
            offset_y=float(self.pos_offset[1]),
        )
        normalized_static = construct_normalized_static_obstacle_from_obj(
            static_obs,
            grid_size=self.grid_size,
            mult_x=self.mult_x,
            mult_y=self.mult_y,
            offset_x=float(self.pos_offset[0]),
            offset_y=float(self.pos_offset[1]),
        )
        normalized_prefer_region = construct_normalized_prefer_region(
            terrain_obs,
            grid_size=self.grid_size,
            mult_x=self.mult_x,
            mult_y=self.mult_y,
            offset_x=float(self.pos_offset[0]),
            offset_y=float(self.pos_offset[1]),
        )

        dynamic_obs_tensor, dynamic_obs_mask = self.process_obs_helper.get_obs_cond(normalized_dynamic)
        static_obs_tensor, static_obs_mask = self.process_obs_helper.get_static_cond(normalized_static)
        prefer_obs_tensor, prefer_obs_mask = self.process_obs_helper.get_region_cond(normalized_prefer_region)

        dynamic_obs_tensor = dynamic_obs_tensor.to(self.device)
        dynamic_obs_mask = dynamic_obs_mask.to(self.device)
        static_obs_tensor = static_obs_tensor.to(self.device)
        static_obs_mask = static_obs_mask.to(self.device)
        prefer_obs_tensor = prefer_obs_tensor.to(self.device)
        prefer_obs_mask = prefer_obs_mask.to(self.device)

        context_cond = {
            "dynamic_obs_encoder": {
                "mask": dynamic_obs_mask.unsqueeze(0),
                "input_vals": [obs.unsqueeze(0) for obs in dynamic_obs_tensor],
            },
            "static_obs_encoder": {
                "mask": static_obs_mask.unsqueeze(0),
                "input_vals": [obs.unsqueeze(0) for obs in static_obs_tensor],
            },
            "goal_encoder": {
                "mask": torch.ones([1, 1], device=self.device).float(),
                "input_vals": torch.tensor([self.goal_pos_normalized], device=self.device).float(),
            },
            "terrain_encoder": {
                "mask": prefer_obs_mask.unsqueeze(0),
                "input_vals": [obs.unsqueeze(0) for obs in prefer_obs_tensor],
            },
        }
        context_cond = repeat_context(context_cond, 2 * self.total_batch_size)
        state_cond = {"0": torch.tensor([self.pos_offset_normalized], device=self.device).float()}
        return context_cond, state_cond

    def _prepare_replan_state(self, state_cond):
        if self.diffusion_path is None:
            self.curr_diff_state_idx = 0
            return None, None, state_cond

        with self.past_odoms_lock:
            if len(self.past_odoms) < 2:
                self.curr_diff_state_idx = 0
                return None, None, state_cond
            past_path = np.asarray(self.past_odoms, dtype=np.float32)[:, :2]

        add_noise_step = self.replan_add_noise_step
        n_timesteps = add_noise_step + 1
        t = make_timesteps(add_noise_step, self.total_batch_size, self.device)
        tmp_diffusion_path = self.diffusion_path.repeat(self.total_batch_size, 1, 1)
        xt = self.anchor_model.q_sample(tmp_diffusion_path, t)

        interp_past_path = fast_interpolate_path(past_path, self.odometry_dt, self.diffusion_planning_dt)
        interp_past_path[:, 0] = self._scale_x(interp_past_path[:, 0])
        interp_past_path[:, 1] = self._scale_y(interp_past_path[:, 1])
        interp_past_path = self.path_normalizer.normalize(interp_past_path)
        interp_past_path = torch.tensor(interp_past_path, device=self.device).float().unsqueeze(0)

        state_cond = {str(idx): interp_past_path[:, idx] for idx in range(interp_past_path.shape[1])}
        self.curr_diff_state_idx = max(int(interp_past_path.shape[1]) - 2, 0)
        return xt, n_timesteps, state_cond

    def _replan(self) -> None:
        if self.start_pose is None:
            return

        dynamic_obs, dynamic_model_keys = self._build_dynamic_obstacles()
        tasks = self._build_tasks(dynamic_model_keys)
        context_cond, state_cond = self._context_constructor(dynamic_obs, self.static_obs, self.terrain_obs)

        context_conds = []
        model_keys_used = []
        for task in tasks:
            if task["context_field"] == "goal_encoder":
                continue
            model_keys_used.append(task["model_key"])
            task_copy = dict(task)
            task_copy["obs_idx"] = torch.tensor(task_copy["obs_idx"])
            _, model_context = construct_context_state_conds(self.diffusion_models, context_cond, task_copy)
            context_conds.append(model_context)

        if not context_conds:
            self.get_logger().warning("No valid diffusion tasks were built for the current scene.")
            return

        stacked_context_cond = stack_dictionaries_list(context_conds)
        models = [self.diffusion_models[key].model for key in model_keys_used]
        params, buffers = stack_module_state(models)
        base_model = copy.deepcopy(self.anchor_model.model).to("meta")
        cfg_mask_cond = torch.ones(self.total_batch_size, device=self.device)
        cfg_mask_uncond = torch.zeros(self.total_batch_size, device=self.device)
        cfg_mask = torch.cat([cfg_mask_cond, cfg_mask_uncond], dim=0).unsqueeze(0).repeat(len(model_keys_used), 1)

        xt, n_timesteps, state_cond = self._prepare_replan_state(state_cond)
        chain = get_guided_diffusion_path_vmap(
            stacked_context_cond,
            self.anchor_model,
            base_model,
            params,
            buffers,
            len(model_keys_used),
            cfg_mask,
            state_cond,
            n_timesteps,
            xt,
            self.max_planning_time,
            self.total_batch_size,
            self.guidance_weight,
            self.device,
            compile=self.compile,
        )
        self._set_diffusion(chain)
        self._publish_path()

    def _set_diffusion(self, chain) -> None:
        if self.diffusion_path is None:
            self.diffusion_path = chain[-1][:1]
            return

        traj_segments = chain[-1][:, self.curr_diff_state_idx : self.curr_diff_state_idx + 10]
        segment = self.diffusion_path[:, self.curr_diff_state_idx : self.curr_diff_state_idx + 10]
        distances = ((traj_segments - segment) ** 2).mean(dim=(1, 2))
        min_index = torch.argmin(distances)
        self.diffusion_path = chain[-1][min_index.item() : min_index.item() + 1]

    def _publish_path(self) -> None:
        if self.diffusion_path is None or self.start_pose is None:
            return

        local_path = self.diffusion_path.detach().cpu().numpy()[0].copy()
        local_path = self.path_normalizer.unnormalize(local_path)
        local_path[:, 0] = self._descale_x(local_path[:, 0]) - float(self.pos_offset[0])
        local_path[:, 1] = self._descale_y(local_path[:, 1]) - float(self.pos_offset[1])

        idx = find_first_waypoint_within_radius(local_path, self.goal_pos, self.goal_radius)
        if idx != -1:
            local_path = local_path[: idx + 2]

        legacy = Float32MultiArray()
        legacy.data = [float(self.curr_diff_state_idx)] + local_path.astype(np.float32).flatten().tolist()
        self.legacy_path_pub.publish(legacy)

        path_msg = PathMsg()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.output_frame_id or self.latest_odom_frame_id or "odom"
        global_waypoints = []
        for waypoint in local_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            global_xytheta = coordinate_transform_to_global(self.start_pose, [float(waypoint[0]), float(waypoint[1]), 0.0])
            global_waypoints.append(global_xytheta)
            pose.pose.position.x = float(global_xytheta[0])
            pose.pose.position.y = float(global_xytheta[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self._publish_markers(path_msg.header.frame_id, global_waypoints)

    def _publish_markers(self, frame_id: str, global_waypoints: List[List[float]]) -> None:
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = frame_id
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        if not global_waypoints:
            self.marker_pub.publish(marker_array)
            return

        stamp = self.get_clock().now().to_msg()

        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = stamp
        line_marker.ns = "prefnav_diffusion"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.08
        line_marker.color.r = 0.1
        line_marker.color.g = 0.8
        line_marker.color.b = 1.0
        line_marker.color.a = 0.95
        line_marker.points = [self._make_point(waypoint) for waypoint in global_waypoints]
        marker_array.markers.append(line_marker)

        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = frame_id
        waypoint_marker.header.stamp = stamp
        waypoint_marker.ns = "prefnav_diffusion"
        waypoint_marker.id = 1
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.pose.orientation.w = 1.0
        waypoint_marker.scale.x = 0.18
        waypoint_marker.scale.y = 0.18
        waypoint_marker.scale.z = 0.18
        waypoint_marker.color.r = 0.0
        waypoint_marker.color.g = 0.55
        waypoint_marker.color.b = 1.0
        waypoint_marker.color.a = 0.9
        waypoint_marker.points = [self._make_point(waypoint) for waypoint in global_waypoints]
        marker_array.markers.append(waypoint_marker)

        current_idx = min(max(int(self.curr_diff_state_idx), 0), len(global_waypoints) - 1)
        current_marker = Marker()
        current_marker.header.frame_id = frame_id
        current_marker.header.stamp = stamp
        current_marker.ns = "prefnav_diffusion"
        current_marker.id = 2
        current_marker.type = Marker.SPHERE
        current_marker.action = Marker.ADD
        current_marker.pose.position = self._make_point(global_waypoints[current_idx])
        current_marker.pose.orientation.w = 1.0
        current_marker.scale.x = 0.28
        current_marker.scale.y = 0.28
        current_marker.scale.z = 0.28
        current_marker.color.r = 1.0
        current_marker.color.g = 0.4
        current_marker.color.b = 0.1
        current_marker.color.a = 1.0
        marker_array.markers.append(current_marker)

        marker_array.markers.append(self._make_goal_marker(frame_id, stamp))
        self.marker_pub.publish(marker_array)

    def _make_goal_marker(self, frame_id: str, stamp):
        goal_global = coordinate_transform_to_global(
            self.start_pose,
            [float(self.goal_pos[0]), float(self.goal_pos[1]), 0.0],
        )
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "prefnav_diffusion"
        marker.id = 3
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position = self._make_point(goal_global)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * self.goal_radius
        marker.scale.y = 2.0 * self.goal_radius
        marker.scale.z = 0.05
        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.25
        return marker

    @staticmethod
    def _make_point(xytheta: List[float]) -> Point:
        point = Point()
        point.x = float(xytheta[0])
        point.y = float(xytheta[1])
        point.z = 0.0
        return point

    def _scale_x(self, x):
        return (x + self.grid_size / 2.0) * self.mult_x - (self.grid_size / 2.0)

    def _scale_y(self, y):
        return y * self.mult_y

    def _descale_x(self, x):
        return (x + self.grid_size / 2.0) / self.mult_x - (self.grid_size / 2.0)

    def _descale_y(self, y):
        return y / self.mult_y


def main() -> None:
    rclpy.init()
    node = DiffusionWaypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
