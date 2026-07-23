#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point, TransformStamped, Vector3
from people_detector.msg import MapPerson, MapPersonArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# C++ composable publishes ``hdl_people_tracking/msg/TrackArray`` (not
# ``hdl_people_tracking_msgs``). Subscribing with the wrong type receives
# nothing; prefer the matching package first.
try:
    from hdl_people_tracking.msg import TrackArray as HdlTrackArray
except ImportError:
    HdlTrackArray = None

try:
    from hdl_people_tracking_msgs.msg import TrackArray as HdlMsgsTrackArray
except ImportError:
    HdlMsgsTrackArray = None


Matrix3 = List[List[float]]
VectorTuple = Tuple[float, float, float]


class MapTracksAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("map_tracks_adapter")

        self.declare_parameter("tracks_topic", "/tracks")
        self.declare_parameter("output_topic", "/people/map_tracks")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("source", "hdl_tracks")
        self.declare_parameter("confidence", 1.0)
        self.declare_parameter("transform_timeout_sec", 0.05)

        self._target_frame = self._normalize_frame(str(self.get_parameter("target_frame").value))
        self._source = str(self.get_parameter("source").value)
        self._confidence = float(self.get_parameter("confidence").value)
        self._transform_timeout = float(self.get_parameter("transform_timeout_sec").value)
        self._last_sample_time_by_id: Dict[int, float] = {}

        output_topic = str(self.get_parameter("output_topic").value)
        tracks_topic = str(self.get_parameter("tracks_topic").value)
        self._pub = self.create_publisher(MapPersonArray, output_topic, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        if HdlTrackArray is not None:
            self.create_subscription(HdlTrackArray, tracks_topic, self._on_tracks, 10)
            self.get_logger().info(
                f"Map tracks subscribed as hdl_people_tracking/TrackArray on {tracks_topic}"
            )
        elif HdlMsgsTrackArray is not None:
            self.create_subscription(HdlMsgsTrackArray, tracks_topic, self._on_tracks, 10)
            self.get_logger().warn(
                f"hdl_people_tracking Python not found; using hdl_people_tracking_msgs/TrackArray "
                f"on {tracks_topic} — this will NOT receive data from the HDL composable "
                f"(install package hdl_people_tracking and rebuild people_detector)."
            )
        else:
            self.get_logger().error(
                "No TrackArray message type available (hdl_people_tracking / "
                "hdl_people_tracking_msgs). Map tracks adapter will not receive /tracks."
            )

        self.get_logger().info(f"Publishing map-frame tracks on: {output_topic}")

    def _on_tracks(self, msg) -> None:
        source_frame = self._normalize_frame(msg.header.frame_id)
        if not source_frame:
            self.get_logger().warn("Received /tracks with an empty header.frame_id; skipping.", throttle_duration_sec=5.0)
            return

        transform = self._lookup_transform(source_frame, msg.header.stamp)
        if transform is None:
            return

        rotation, translation = transform
        sample_time_sec = self._stamp_to_sec(msg.header.stamp)

        out = MapPersonArray()
        out.header = msg.header
        out.header.frame_id = self._target_frame

        for track in msg.tracks:
            track_id = int(track.id)

            person = MapPerson()
            person.id = track_id
            person.source = self._source
            person.confidence = self._confidence
            person.sample_time_sec = sample_time_sec
            person.track_age_sec = max(0.0, float(track.age))
            person.dt_sec = self._dt_for_track(track_id, sample_time_sec)
            person.position = self._transform_point(track.pos, rotation, translation)
            person.velocity = self._transform_vector(track.vel, rotation)
            person.size = self._track_size(track)
            person.position_covariance = self._rotate_covariance(track.pos_cov, rotation)
            person.velocity_covariance = self._rotate_covariance(track.vel_cov, rotation)
            out.people.append(person)

        self._pub.publish(out)

    def _lookup_transform(self, source_frame: str, stamp) -> Optional[Tuple[Matrix3, VectorTuple]]:
        if source_frame == self._target_frame:
            return (self._identity(), (0.0, 0.0, 0.0))

        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self._transform_timeout),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"TF unavailable for /tracks: {source_frame} -> {self._target_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

        return self._transform_to_matrix(tf)

    def _dt_for_track(self, track_id: int, sample_time_sec: float) -> float:
        previous = self._last_sample_time_by_id.get(track_id)
        self._last_sample_time_by_id[track_id] = sample_time_sec
        if previous is None:
            return 0.0
        return max(0.0, sample_time_sec - previous)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _track_size(track) -> Vector3:
        if track.associated:
            return track.associated[0].size
        return Vector3()

    @staticmethod
    def _transform_to_matrix(tf: TransformStamped) -> Tuple[Matrix3, VectorTuple]:
        q = tf.transform.rotation
        rotation = MapTracksAdapterNode._rotation_from_quaternion(q.x, q.y, q.z, q.w)
        t = tf.transform.translation
        return (rotation, (float(t.x), float(t.y), float(t.z)))

    @staticmethod
    def _rotation_from_quaternion(x: float, y: float, z: float, w: float) -> Matrix3:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return MapTracksAdapterNode._identity()

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        return [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]

    @staticmethod
    def _transform_point(point: Point, rotation: Matrix3, translation: VectorTuple) -> Point:
        x, y, z = MapTracksAdapterNode._mat_vec(rotation, (point.x, point.y, point.z))
        out = Point()
        out.x = x + translation[0]
        out.y = y + translation[1]
        out.z = z + translation[2]
        return out

    @staticmethod
    def _transform_vector(vector: Vector3, rotation: Matrix3) -> Vector3:
        x, y, z = MapTracksAdapterNode._mat_vec(rotation, (vector.x, vector.y, vector.z))
        out = Vector3()
        out.x = x
        out.y = y
        out.z = z
        return out

    @staticmethod
    def _rotate_covariance(covariance: Sequence[float], rotation: Matrix3) -> List[float]:
        cov = list(covariance)
        if len(cov) != 9:
            cov = [0.0] * 9

        matrix = [
            [float(cov[0]), float(cov[1]), float(cov[2])],
            [float(cov[3]), float(cov[4]), float(cov[5])],
            [float(cov[6]), float(cov[7]), float(cov[8])],
        ]
        rotated = [[0.0 for _ in range(3)] for _ in range(3)]
        for i in range(3):
            for j in range(3):
                rotated[i][j] = sum(
                    rotation[i][k] * matrix[k][l] * rotation[j][l]
                    for k in range(3)
                    for l in range(3)
                )

        return [rotated[i][j] for i in range(3) for j in range(3)]

    @staticmethod
    def _mat_vec(matrix: Matrix3, vector: VectorTuple) -> VectorTuple:
        return (
            matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
            matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
            matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
        )

    @staticmethod
    def _identity() -> Matrix3:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _normalize_frame(frame: str) -> str:
        return frame.strip().lstrip("/")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapTracksAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
