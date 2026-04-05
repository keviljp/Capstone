from __future__ import annotations

from typing import Dict, Set

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .common import covariance_from_variances, ensure_robot_ns, load_tag_map


class TagRouter(Node):
    def __init__(self) -> None:
        super().__init__('tag_router')

        self.declare_parameter('tag_map_file', '')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('x_variance', 0.01)
        self.declare_parameter('y_variance', 0.01)
        self.declare_parameter('yaw_variance', 0.03)
        self.declare_parameter('ignore_unknown_tags', True)

        tag_map_file = str(self.get_parameter('tag_map_file').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.ignore_unknown_tags = bool(self.get_parameter('ignore_unknown_tags').value)

        if not tag_map_file:
            raise RuntimeError('tag_map_file parameter is required.')

        self.tag_map: Dict[int, str] = load_tag_map(tag_map_file)
        self.publishers: Dict[str, any] = {}
        self.discovered: Set[str] = set()

        x_var = float(self.get_parameter('x_variance').value)
        y_var = float(self.get_parameter('y_variance').value)
        yaw_var = float(self.get_parameter('yaw_variance').value)
        self.pose_covariance = covariance_from_variances(
            x_var=x_var,
            y_var=y_var,
            z_var=1e6,
            roll_var=1e6,
            pitch_var=1e6,
            yaw_var=yaw_var,
        )

        self.discovery_pub = self.create_publisher(String, '/swarm_state/discovered_robot_id', 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            10,
        )

        self.get_logger().info(f'Tag router listening on {self.detections_topic}')
        self.get_logger().info(f'Loaded {len(self.tag_map)} tag-to-robot mappings.')

    def _publisher_for(self, robot_id: str):
        robot_ns = ensure_robot_ns(robot_id)
        topic = f'{robot_ns}/tag_pose'
        if topic not in self.publishers:
            self.publishers[topic] = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        return self.publishers[topic]

    def _frame_name(self, family: str, tag_id: int) -> str:
        family = family.strip()
        if family.startswith('tag'):
            return f'{family}:{tag_id}'
        return f'tag{family}:{tag_id}'

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        for detection in msg.detections:
            tag_id = int(detection.id)
            robot_id = self.tag_map.get(tag_id)

            if not robot_id:
                if not self.ignore_unknown_tags:
                    self.get_logger().warning(f'Ignoring unmapped tag id {tag_id}')
                continue

            tag_frame = self._frame_name(detection.family, tag_id)
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    tag_frame,
                    rclpy.time.Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as exc:
                self.get_logger().debug(f'TF lookup failed for {tag_frame}: {exc}')
                continue

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = self.map_frame
            pose_msg.pose.pose.position.x = tf_msg.transform.translation.x
            pose_msg.pose.pose.position.y = tf_msg.transform.translation.y
            pose_msg.pose.pose.position.z = 0.0
            pose_msg.pose.pose.orientation = tf_msg.transform.rotation
            pose_msg.pose.covariance = list(self.pose_covariance)

            self._publisher_for(robot_id).publish(pose_msg)

            if robot_id not in self.discovered:
                self.discovered.add(robot_id)
                notice = String()
                notice.data = robot_id
                self.discovery_pub.publish(notice)
                self.get_logger().info(f'Discovered robot {robot_id} from tag {tag_id}; UKF may now be started.')


def main() -> None:
    rclpy.init()
    node = TagRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
