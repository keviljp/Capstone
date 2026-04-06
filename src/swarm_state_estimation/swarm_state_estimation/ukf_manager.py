from __future__ import annotations

from pathlib import Path
import os
import signal
import subprocess
import tempfile
from typing import Dict
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import ensure_robot_ns


class UkfManager(Node):
    def __init__(self) -> None:
        super().__init__('ukf_manager')

        self.declare_parameter('ukf_template_file', '')
        self.declare_parameter('discovery_topic', '/swarm_state/discovered_robot_id')
        self.declare_parameter('robot_prefix', 'swarmbot_')
        self.declare_parameter('generated_config_dir', '')

        self.ukf_template_file = str(self.get_parameter('ukf_template_file').value)
        self.discovery_topic = str(self.get_parameter('discovery_topic').value)
        generated_dir = str(self.get_parameter('generated_config_dir').value).strip()

        if not self.ukf_template_file:
            raise RuntimeError('ukf_template_file parameter is required.')

        if generated_dir:
            self.generated_dir = Path(generated_dir)
            self.generated_dir.mkdir(parents=True, exist_ok=True)
        else:
            self.generated_dir = Path(tempfile.mkdtemp(prefix='swarm_ukf_'))

        with open(self.ukf_template_file, 'r', encoding='utf-8') as f:
            self.template = yaml.safe_load(f) or {}

        self.processes: Dict[str, subprocess.Popen] = {}
        self.sub = self.create_subscription(String, self.discovery_topic, self._on_robot_discovered, 10)
        self.get_logger().info(f'UKF manager ready. Generated configs go to {self.generated_dir}')

    def _make_config(self, robot_id: str) -> Path:
        robot_ns = ensure_robot_ns(robot_id)
        ns_stripped = robot_ns.strip('/')
        base_link_frame = f'{ns_stripped}/base_link'
        odom_frame = f'{ns_stripped}/odom_frame'

        cfg = yaml.safe_load(yaml.safe_dump(self.template))
        params = cfg.setdefault('ukf_filter_node', {}).setdefault('ros__parameters', {})
        params['map_frame'] = 'map'
        params['odom_frame'] = odom_frame
        params['base_link_frame'] = base_link_frame
        params['world_frame'] = 'map'
        params['pose0'] = 'tag_pose'
        params['imu0'] = 'imu/data'

        out = self.generated_dir / f'ukf_{robot_id}.yaml'
        with out.open('w', encoding='utf-8') as f:
            yaml.safe_dump(cfg, f, sort_keys=False)
        return out

    def _on_robot_discovered(self, msg: String) -> None:
        robot_id = msg.data.strip().lower()
        if not robot_id:
            return
        if robot_id in self.processes:
            return

        config_file = self._make_config(robot_id)
        robot_ns = ensure_robot_ns(robot_id)
        cmd = [
            'ros2', 'run', 'robot_localization', 'ukf_node',
            '--ros-args',
            '-r', '__ns:=' + robot_ns,
            '-r', '__node:=ukf_filter_node',
            '-r', 'odometry/filtered:=odom',
            '--params-file', str(config_file),
        ]

        self.get_logger().info(f"Starting UKF for {robot_id}: {' '.join(cmd)}")
        proc = subprocess.Popen(cmd)
        self.processes[robot_id] = proc

    def destroy_node(self) -> bool:
        for robot_id, proc in self.processes.items():
            if proc.poll() is None:
                self.get_logger().info(f'Shutting down UKF for {robot_id}')
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = UkfManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
