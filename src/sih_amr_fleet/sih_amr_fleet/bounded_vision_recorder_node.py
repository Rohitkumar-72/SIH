import json
import os
import pathlib
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sih_amr_interfaces.msg import RobotState

from .common import FLEET_STATE_QOS, POSE_QOS, now_seconds


class BoundedVisionRecorderNode(Node):
    """Bounded, rate-limited camera recorder with synchronized pose and storage quota."""

    def __init__(self):
        super().__init__('bounded_vision_recorder_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.enabled = self.declare_parameter('record_camera_images', False).value
        self.output_dir = self.declare_parameter('output_dir', f'/tmp/vision_dataset/{self.robot_id}').value
        self.max_storage_mb = self.declare_parameter('max_storage_quota_mb', 200.0).value
        self.sample_interval_s = self.declare_parameter('sample_interval_s', 1.0).value

        self.last_sample_time = 0.0
        self.current_storage_bytes = 0
        self.quota_exceeded = False
        self.current_pose = None
        self.frame_index = 0
        self.index_file = None

        if self.enabled:
            self.out_path = pathlib.Path(self.output_dir)
            self.out_path.mkdir(parents=True, exist_ok=True)
            self.index_path = self.out_path / 'frames_manifest.jsonl'
            self.index_file = open(self.index_path, 'a', buffering=1, encoding='utf-8')

            self.create_subscription(RobotState, 'state', self.on_state, POSE_QOS)
            self.create_subscription(Image, 'camera/image_raw', self.on_image, POSE_QOS)
            self.get_logger().info(f'BoundedVisionRecorderNode started for {self.robot_id} -> {self.output_dir}')

    def on_state(self, msg):
        self.current_pose = {
            'x': float(msg.pose.x),
            'y': float(msg.pose.y),
            'theta': float(msg.pose.theta),
            'timestamp': now_seconds(self)
        }

    def on_image(self, msg):
        if self.quota_exceeded:
            return

        now = now_seconds(self)
        if now - self.last_sample_time < self.sample_interval_s:
            return
        self.last_sample_time = now

        # Enforce quota
        if self.current_storage_bytes >= self.max_storage_mb * 1024 * 1024:
            self.quota_exceeded = True
            self.get_logger().warning(f'Storage quota ({self.max_storage_mb} MB) reached for {self.robot_id}. Stopping recording.')
            return

        self.frame_index += 1
        frame_filename = f'frame_{self.frame_index:06d}.raw'
        frame_filepath = self.out_path / frame_filename

        try:
            # Write frame bytes
            data_bytes = bytes(msg.data)
            frame_filepath.write_bytes(data_bytes)
            self.current_storage_bytes += len(data_bytes)

            # Write manifest index record
            record = {
                'frame_id': self.frame_index,
                'file_name': frame_filename,
                'robot_id': self.robot_id,
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'timestamp_s': now,
                'robot_pose': self.current_pose,
            }
            if self.index_file:
                self.index_file.write(json.dumps(record) + '\n')
        except Exception as e:
            self.get_logger().error(f'Failed writing vision frame: {e}')

    def destroy_node(self):
        if self.index_file:
            try:
                self.index_file.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BoundedVisionRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
