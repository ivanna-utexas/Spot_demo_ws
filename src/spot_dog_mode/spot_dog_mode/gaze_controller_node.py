#!/usr/bin/env python3
"""Auto Dog Mode gaze controller.

Consumes standardized people detections (people_detector/PeopleArray, odom
frame), selects a target person, and drives a dog-like "greet + track" gaze
by publishing body orientation offsets on the spot_ros2 `body_pose` topic.

State machine: IDLE -> GREET -> TRACK -> RELAX -> IDLE.

Only commands the body while Spot is standing still, dog mode is enabled,
and the teleop manual deadman is not held.
"""

import math
import time
from enum import Enum
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose
from people_detector.msg import PeopleArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from spot_msgs.msg import Feedback
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


class GazeState(Enum):
    IDLE = 0
    GREET = 1
    TRACK = 2
    RELAX = 3


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _yaw_pitch_to_quaternion(pose: Pose, pitch: float, yaw: float) -> None:
    """Fill pose.orientation from pitch/yaw (roll = 0). Same convention as
    spot_joy/teleop_node.py body pose control."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    pose.orientation.w = cp * cy
    pose.orientation.x = -sp * sy
    pose.orientation.y = sp * cy
    pose.orientation.z = cp * sy


def _transform_point(tf, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Apply a geometry_msgs TransformStamped to a point (quaternion rotation
    + translation) without requiring tf2_geometry_msgs."""
    q = tf.transform.rotation
    t = tf.transform.translation

    # Rotate v by q: v' = v + 2*qv x (qv x v + qw*v)
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    ux, uy, uz = (
        qy * z - qz * y,
        qz * x - qx * z,
        qx * y - qy * x,
    )
    ux, uy, uz = ux + qw * x, uy + qw * y, uz + qw * z
    rx = x + 2.0 * (qy * uz - qz * uy)
    ry = y + 2.0 * (qz * ux - qx * uz)
    rz = z + 2.0 * (qx * uy - qy * ux)

    return (rx + t.x, ry + t.y, rz + t.z)


class GazeControllerNode(Node):
    def __init__(self):
        super().__init__('dog_mode_gaze_controller')

        # ── Topics / frames ──────────────────────────────────────────────
        self.declare_parameter('people_topic', '/people_detections')
        self.declare_parameter('body_pose_topic', 'body_pose')
        self.declare_parameter('feedback_topic', '/status/feedback')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('body_frame', 'body')

        # ── Behavior tuning ──────────────────────────────────────────────
        self.declare_parameter('command_rate', 10.0)
        self.declare_parameter('attention_radius', 4.0)      # m, person enters attention
        self.declare_parameter('target_switch_margin', 0.75) # m closer before stealing target
        self.declare_parameter('target_match_radius', 1.0)   # m, re-associate target without id
        self.declare_parameter('target_lost_timeout', 1.5)   # s without target -> RELAX
        self.declare_parameter('detection_timeout', 1.0)     # s without any detections msg -> stale
        self.declare_parameter('greet_ramp', 0.5)            # s ramp into greet pose
        self.declare_parameter('greet_hold', 1.2)            # s holding the look-up
        self.declare_parameter('greet_pitch_boost', 0.15)    # rad extra look-up during greet
        self.declare_parameter('smoothing_tau', 0.3)         # s low-pass time constant (TRACK)
        self.declare_parameter('max_rate_rad_s', 1.0)        # rad/s slew limit
        self.declare_parameter('relax_duration', 1.0)        # s ease back to neutral
        self.declare_parameter('max_yaw', 0.4)               # rad, matches teleop MAX_YAW
        self.declare_parameter('max_pitch', 0.4)             # rad, matches teleop MAX_PITCH
        # spot_ros2 body_pose convention: positive pitch = nose down, so
        # looking up at a person is negative pitch. Flip to 1.0 if the robot
        # bows instead of looking up.
        self.declare_parameter('pitch_sign', -1.0)
        self.declare_parameter('default_person_height', 1.7) # m, fallback head height
        self.declare_parameter('min_person_height', 0.5)     # m, below this cluster size is degenerate
        self.declare_parameter('body_height_above_ground', 0.52)  # m, Spot body z when standing

        # ── Integration ──────────────────────────────────────────────────
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('deadman_button', 4)          # L1, matches spot_joy BTN_DEADMAN_MANUAL
        self.declare_parameter('publish_eye_gaze', False)
        self.declare_parameter('eye_gaze_topic', 'eye_animation/gaze')

        self._enabled = bool(self.get_parameter('start_enabled').value)

        # ── State ────────────────────────────────────────────────────────
        self._state = GazeState.IDLE
        self._state_since = time.monotonic()
        self._people_msg: Optional[PeopleArray] = None
        self._people_rx_time = 0.0
        self._standing = False
        self._moving = False
        self._deadman_held = False
        self._target_id: Optional[int] = None
        self._target_body_xyz: Optional[Tuple[float, float, float]] = None
        self._last_target_pos: Optional[Tuple[float, float]] = None
        self._target_head_z: float = float(self.get_parameter('default_person_height').value)
        self._target_last_seen = 0.0
        self._cmd_yaw = 0.0
        self._cmd_pitch = 0.0
        self._commanding = False   # we have published a non-neutral pose
        self._relax_start: Tuple[float, float] = (0.0, 0.0)
        self._tf_warned = False

        # ── ROS interfaces ───────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PeopleArray, self.get_parameter('people_topic').value, self._on_people, 10
        )
        self.create_subscription(
            Feedback, self.get_parameter('feedback_topic').value, self._on_feedback, 10
        )
        self.create_subscription(
            Joy, self.get_parameter('joy_topic').value, self._on_joy, 10
        )

        self.body_pose_pub = self.create_publisher(
            Pose, self.get_parameter('body_pose_topic').value, 1
        )

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.enabled_pub = self.create_publisher(Bool, '/dog_mode/enabled', latched)
        self._publish_enabled()

        self.create_service(SetBool, '/dog_mode/enable', self._on_enable)

        self._eye_gaze_pub = None
        if bool(self.get_parameter('publish_eye_gaze').value):
            try:
                from spot_eye_animation_msgs.msg import Gaze
                self._gaze_msg_type = Gaze
                self._eye_gaze_pub = self.create_publisher(
                    Gaze, self.get_parameter('eye_gaze_topic').value, 10
                )
            except ImportError:
                self.get_logger().warn(
                    'publish_eye_gaze=true but spot_eye_animation_msgs not available'
                )

        rate = float(self.get_parameter('command_rate').value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'Dog mode gaze controller started (enabled={self._enabled}). '
            f'Toggle via /dog_mode/enable.'
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_people(self, msg: PeopleArray) -> None:
        self._people_msg = msg
        self._people_rx_time = time.monotonic()

    def _on_feedback(self, msg: Feedback) -> None:
        self._standing = msg.standing
        self._moving = msg.moving

    def _on_joy(self, msg: Joy) -> None:
        idx = int(self.get_parameter('deadman_button').value)
        self._deadman_held = bool(msg.buttons[idx]) if idx < len(msg.buttons) else False

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self._enabled = request.data
        self._publish_enabled()
        self.get_logger().info(f'Dog mode {"ENABLED" if self._enabled else "DISABLED"}')
        response.success = True
        response.message = f'dog mode enabled={self._enabled}'
        return response

    def _publish_enabled(self) -> None:
        self.enabled_pub.publish(Bool(data=self._enabled))

    # ── Main control loop ────────────────────────────────────────────────

    def _tick(self) -> None:
        now = time.monotonic()

        if not self._gaze_allowed(now):
            self._drop_to_idle()
            return

        self._update_target(now)

        if self._state == GazeState.IDLE:
            if self._target_body_xyz is not None:
                self._transition(GazeState.GREET)
            else:
                self._publish_eye_gaze(detected=False)
            return

        if self._target_body_xyz is None and self._state in (GazeState.GREET, GazeState.TRACK):
            if (now - self._target_last_seen) > float(self.get_parameter('target_lost_timeout').value):
                self._target_id = None
                self._relax_start = (self._cmd_yaw, self._cmd_pitch)
                self._transition(GazeState.RELAX)

        state_t = now - self._state_since

        if self._state == GazeState.GREET:
            self._run_greet(state_t)
        elif self._state == GazeState.TRACK:
            self._run_track()
        elif self._state == GazeState.RELAX:
            self._run_relax(state_t)

    def _gaze_allowed(self, now: float) -> bool:
        if not self._enabled or self._deadman_held:
            return False
        if not self._standing or self._moving:
            return False
        if (now - self._people_rx_time) > float(self.get_parameter('detection_timeout').value):
            return False
        return True

    def _drop_to_idle(self) -> None:
        """Deactivate: return to neutral once and stop commanding."""
        if self._commanding:
            self._publish_pose(0.0, 0.0)
            self._commanding = False
        self._cmd_yaw = 0.0
        self._cmd_pitch = 0.0
        self._target_id = None
        self._target_body_xyz = None
        if self._state != GazeState.IDLE:
            self._transition(GazeState.IDLE)

    # ── Target selection ─────────────────────────────────────────────────

    def _update_target(self, now: float) -> None:
        """Transform detections into body frame and pick/keep a target."""
        msg = self._people_msg
        self._target_body_xyz = None
        if msg is None or not msg.people:
            return

        body_frame = self.get_parameter('body_frame').value
        try:
            tf = self.tf_buffer.lookup_transform(
                body_frame, msg.header.frame_id, rclpy.time.Time()
            )
            self._tf_warned = False
        except Exception as e:
            if not self._tf_warned:
                self.get_logger().warn(
                    f'TF {msg.header.frame_id}->{body_frame} unavailable: {e}'
                )
                self._tf_warned = True
            return

        radius = float(self.get_parameter('attention_radius').value)
        candidates = []  # (dist, id, (x, y, z), size_z)
        for person in msg.people:
            x, y, z = _transform_point(
                tf, person.position.x, person.position.y, person.position.z
            )
            dist = math.hypot(x, y)
            if dist <= radius:
                candidates.append((dist, int(person.id), (x, y, z), float(person.size.z)))

        if not candidates:
            return

        candidates.sort(key=lambda c: c[0])
        nearest = candidates[0]

        chosen = None
        if self._target_id is not None:
            # Sticky target: prefer the current target if still present.
            current = next((c for c in candidates if c[1] == self._target_id), None)
            if current is None:
                # Fall back to spatial re-association (sources without stable ids).
                match_r = float(self.get_parameter('target_match_radius').value)
                if self._last_target_pos is not None:
                    lx, ly = self._last_target_pos
                    current = min(
                        (c for c in candidates
                         if math.hypot(c[2][0] - lx, c[2][1] - ly) <= match_r),
                        key=lambda c: c[0],
                        default=None,
                    )
            if current is not None:
                margin = float(self.get_parameter('target_switch_margin').value)
                # Keep the current target unless someone is clearly closer.
                chosen = nearest if nearest[0] + margin < current[0] else current

        if chosen is None:
            chosen = nearest

        new_target = self._target_id is None
        self._target_id = chosen[1]
        self._target_body_xyz = chosen[2]
        self._last_target_pos = (chosen[2][0], chosen[2][1])
        self._target_last_seen = now

        # Head height: top of the detected cluster, or a default person height.
        min_h = float(self.get_parameter('min_person_height').value)
        size_z = chosen[3]
        person_z = chosen[2][2]
        if size_z >= min_h:
            self._target_head_z = person_z + size_z / 2.0
        else:
            # Cluster size degenerate: assume an average-height person standing
            # on the ground under the centroid.
            ground_z = -float(self.get_parameter('body_height_above_ground').value)
            self._target_head_z = ground_z + float(self.get_parameter('default_person_height').value)

        if new_target and self._state == GazeState.TRACK:
            # A brand-new person while already tracking: greet them too.
            self._transition(GazeState.GREET)

    # ── State behaviors ──────────────────────────────────────────────────

    def _gaze_angles(self) -> Tuple[float, float]:
        """Geometric yaw/pitch (rad) from body center toward the target head."""
        x, y, _ = self._target_body_xyz
        yaw = math.atan2(y, x)
        horiz = max(math.hypot(x, y), 0.1)
        pitch_up = math.atan2(self._target_head_z, horiz)
        pitch = float(self.get_parameter('pitch_sign').value) * pitch_up

        max_yaw = float(self.get_parameter('max_yaw').value)
        max_pitch = float(self.get_parameter('max_pitch').value)
        return _clamp(yaw, -max_yaw, max_yaw), _clamp(pitch, -max_pitch, max_pitch)

    def _run_greet(self, state_t: float) -> None:
        if self._target_body_xyz is None:
            return
        yaw, pitch = self._gaze_angles()

        boost = float(self.get_parameter('greet_pitch_boost').value)
        pitch = _clamp(
            pitch + float(self.get_parameter('pitch_sign').value) * boost,
            -float(self.get_parameter('max_pitch').value),
            float(self.get_parameter('max_pitch').value),
        )

        ramp = float(self.get_parameter('greet_ramp').value)
        hold = float(self.get_parameter('greet_hold').value)
        alpha = _clamp(state_t / max(ramp, 1e-3), 0.0, 1.0)

        self._cmd_yaw = alpha * yaw
        self._cmd_pitch = alpha * pitch
        self._publish_pose(self._cmd_pitch, self._cmd_yaw)
        self._publish_eye_gaze(detected=True)

        if state_t >= ramp + hold:
            self._transition(GazeState.TRACK)

    def _run_track(self) -> None:
        if self._target_body_xyz is None:
            # Target briefly missing; hold last pose until lost-timeout fires.
            self._publish_pose(self._cmd_pitch, self._cmd_yaw)
            return
        yaw, pitch = self._gaze_angles()

        tau = max(float(self.get_parameter('smoothing_tau').value), 1e-3)
        alpha = _clamp(self._dt / tau, 0.0, 1.0)
        max_step = float(self.get_parameter('max_rate_rad_s').value) * self._dt

        self._cmd_yaw += _clamp(alpha * (yaw - self._cmd_yaw), -max_step, max_step)
        self._cmd_pitch += _clamp(alpha * (pitch - self._cmd_pitch), -max_step, max_step)
        self._publish_pose(self._cmd_pitch, self._cmd_yaw)
        self._publish_eye_gaze(detected=True)

    def _run_relax(self, state_t: float) -> None:
        duration = max(float(self.get_parameter('relax_duration').value), 1e-3)
        alpha = 1.0 - _clamp(state_t / duration, 0.0, 1.0)
        self._cmd_yaw = self._relax_start[0] * alpha
        self._cmd_pitch = self._relax_start[1] * alpha
        self._publish_pose(self._cmd_pitch, self._cmd_yaw)
        self._publish_eye_gaze(detected=False)

        if state_t >= duration:
            self._commanding = False
            self._transition(GazeState.IDLE)

    def _transition(self, state: GazeState) -> None:
        self.get_logger().info(f'Gaze state: {self._state.name} -> {state.name}')
        self._state = state
        self._state_since = time.monotonic()

    # ── Outputs ──────────────────────────────────────────────────────────

    def _publish_pose(self, pitch: float, yaw: float) -> None:
        pose = Pose()
        _yaw_pitch_to_quaternion(pose, pitch, yaw)
        self.body_pose_pub.publish(pose)
        self._commanding = True

    def _publish_eye_gaze(self, detected: bool) -> None:
        if self._eye_gaze_pub is None:
            return
        msg = self._gaze_msg_type()
        msg.header.stamp = self.get_clock().now().to_msg()
        max_yaw = float(self.get_parameter('max_yaw').value)
        max_pitch = float(self.get_parameter('max_pitch').value)
        # Screen convention: +x = viewer's right = robot's left, +y = down.
        msg.x = _clamp(-self._cmd_yaw / max_yaw, -1.0, 1.0)
        msg.y = _clamp(self._cmd_pitch / max_pitch, -1.0, 1.0)
        msg.detected = detected
        msg.num_faces = 1 if detected else 0
        self._eye_gaze_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
