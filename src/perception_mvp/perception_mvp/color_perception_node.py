import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401


class ColorPerceptionNode(Node):
    def __init__(self):
        super().__init__('color_perception_node')

        self.bridge = CvBridge()

        self.rgb_msg = None
        self.depth_msg = None
        self.camera_info_msg = None
        self._last_missing_log_time = 0.0

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.declare_parameter('target_frame', 'world')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.declare_parameter('camera_frame', 'rgbd_camera_frame')
        self.camera_frame = self.get_parameter(
            'camera_frame').get_parameter_value().string_value

        # 仍然保留参数启动方式，但之后可以被状态机动态切换
        self.declare_parameter('target_name', 'red_cylinder')
        self.target_name = self.get_parameter(
            'target_name').get_parameter_value().string_value

        # 预抓取抬高高度（米）
        self.declare_parameter('pregrasp_height', 0.10)
        self.pregrasp_height = self.get_parameter(
            'pregrasp_height').get_parameter_value().double_value
        self.declare_parameter('bin_a_workspace_min', [-0.55, 0.05, -0.05])
        self.declare_parameter('bin_a_workspace_max', [-0.15, 0.34, 0.35])
        self.bin_a_workspace_min = [
            float(v) for v in self.get_parameter('bin_a_workspace_min').value
        ]
        self.bin_a_workspace_max = [
            float(v) for v in self.get_parameter('bin_a_workspace_max').value
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_configs = {
            'red_cylinder': {
                'hsv_ranges': [
                    ((0, 100, 50), (10, 255, 255)),
                    ((170, 100, 50), (180, 255, 255)),
                ],
                'min_area': 100.0,
                'marker_color': (1.0, 0.0, 0.0),
                'marker_id': 0,
            },
            'yellow_box': {
                'hsv_ranges': [
                    ((20, 80, 80), (40, 255, 255)),
                ],
                'min_area': 150.0,
                'marker_color': (1.0, 1.0, 0.0),
                'marker_id': 1,
            },
            'brown_sphere': {
                'hsv_ranges': [
                    ((5, 60, 30), (25, 255, 200)),
                ],
                'min_area': 120.0,
                'marker_color': (0.59, 0.29, 0.0),
                'marker_id': 2,
            },
        }

        if self.target_name != 'all' and self.target_name not in self.target_configs:
            self.get_logger().warn(
                f'Unknown target_name={self.target_name}, fallback to red_cylinder'
            )
            self.target_name = 'red_cylinder'

        # -------------------------
        # Subscribers
        # -------------------------
        self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.rgb_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Image,
            '/rgbd_camera/depth_image',
            self.depth_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            '/rgbd_camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        # 新增：由状态机动态指定当前目标
        self.create_subscription(
            String,
            '/task/current_target_name',
            self.target_name_callback,
            10
        )

        # -------------------------
        # Publishers
        # -------------------------
        self.point_camera_pubs = {}
        self.point_target_pubs = {}

        for target_name in self.target_configs.keys():
            self.point_camera_pubs[target_name] = self.create_publisher(
                PointStamped,
                f'/perception/{target_name}_point_camera',
                10
            )
            self.point_target_pubs[target_name] = self.create_publisher(
                PointStamped,
                f'/perception/{target_name}_point_target_frame',
                10
            )

        self.marker_pub = self.create_publisher(
            Marker,
            '/perception/target_marker',
            10
        )

        # 统一输出给后续规划/状态机模块
        self.current_visible_pub = self.create_publisher(
            Bool,
            '/perception/current_target_visible',
            10
        )
        self.current_point_world_pub = self.create_publisher(
            PointStamped,
            '/perception/current_target_point_world',
            10
        )
        self.current_pregrasp_world_pub = self.create_publisher(
            PointStamped,
            '/perception/current_target_pregrasp_world',
            10
        )
        self.current_target_name_pub = self.create_publisher(
            String,
            '/perception/current_target_name_echo',
            10
        )

        self.timer = self.create_timer(0.5, self.process)

        self.get_logger().info(
            f'Color perception node started. '
            f'target_frame={self.target_frame}, '
            f'camera_frame={self.camera_frame}, '
            f'initial_target_name={self.target_name}, '
            f'pregrasp_height={self.pregrasp_height:.3f}'
        )

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------
    def rgb_callback(self, msg: Image):
        self.rgb_msg = msg

    def depth_callback(self, msg: Image):
        self.depth_msg = msg

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info_msg = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def target_name_callback(self, msg: String):
        new_name = msg.data.strip()

        if new_name == '':
            self.get_logger().warn('Received empty current_target_name, ignore.')
            return

        if new_name not in self.target_configs:
            self.get_logger().warn(
                f'Unknown current_target_name={new_name}, ignore.'
            )
            return

        if new_name != self.target_name:
            old_name = self.target_name
            self.target_name = new_name

            # 切换目标时，先把统一可见性清为 False，避免上层状态机读到旧目标残留
            self.publish_current_visible(False)

            # 删除旧目标 marker（若存在）
            if old_name in self.target_configs:
                self.delete_marker(self.target_configs[old_name]['marker_id'])

            self.get_logger().info(
                f'switch target_name: {old_name} -> {self.target_name}'
            )

    # --------------------------------------------------
    # Main processing
    # --------------------------------------------------
    def process(self):
        if self.rgb_msg is None or self.depth_msg is None or self.camera_info_msg is None:
            now = time.monotonic()
            if now - self._last_missing_log_time > 5.0:
                missing = []
                if self.rgb_msg is None:
                    missing.append('/rgbd_camera/image')
                if self.depth_msg is None:
                    missing.append('/rgbd_camera/depth_image')
                if self.camera_info_msg is None:
                    missing.append('/rgbd_camera/camera_info')
                self.get_logger().warn(
                    f'process() idling, missing topic(s): {", ".join(missing)} '
                    f'(throttled to 5s; check `ros2 topic hz` on these)'
                )
                self._last_missing_log_time = now
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                self.rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(
                self.depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        depth_m = self.convert_depth_to_meters(depth_image)
        if depth_m is None:
            self.get_logger().error('Unsupported depth image format.')
            return

        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        camera_frame = self.camera_frame
        if camera_frame == '':
            camera_frame = self.depth_msg.header.frame_id or self.rgb_msg.header.frame_id
            camera_frame = camera_frame.replace('::', '/')

        # 主线版本：默认只跟踪当前一个目标
        if self.target_name == 'all':
            active_targets = list(self.target_configs.keys())
        else:
            active_targets = [self.target_name]

        inactive_targets = [
            name for name in self.target_configs.keys() if name not in active_targets
        ]

        for name in inactive_targets:
            self.delete_marker(self.target_configs[name]['marker_id'])

        current_target_found = False

        for target_name in active_targets:
            cfg = self.target_configs[target_name]

            result = self.detect_one_target(
                hsv=hsv,
                depth_m=depth_m,
                camera_frame=camera_frame,
                cfg=cfg
            )

            if result is None:
                self.delete_marker(cfg['marker_id'])
                self.get_logger().info(
                    f'target={target_name}, visible=False'
                )

                if self.target_name != 'all':
                    self.publish_current_visible(False)
                    self.publish_current_target_name_echo(target_name)

                continue

            point_camera = result['point_camera']
            self.point_camera_pubs[target_name].publish(point_camera)

            try:
                point_target = self.tf_buffer.transform(
                    point_camera,
                    self.target_frame,
                    timeout=Duration(seconds=0.2)
                )

                if not self.point_in_bin_a_workspace(point_target):
                    self.delete_marker(cfg['marker_id'])
                    self.get_logger().warn(
                        f'target={target_name}, visible=True, '
                        f'but transformed point is outside Bin A workspace; '
                        f'camera_xyz=({point_camera.point.x:.3f}, '
                        f'{point_camera.point.y:.3f}, '
                        f'{point_camera.point.z:.3f}), '
                        f'{self.target_frame}_xyz=({point_target.point.x:.3f}, '
                        f'{point_target.point.y:.3f}, '
                        f'{point_target.point.z:.3f}), '
                        f'min={self.bin_a_workspace_min}, '
                        f'max={self.bin_a_workspace_max}'
                    )
                    if self.target_name != 'all':
                        self.publish_current_visible(False)
                        self.publish_current_target_name_echo(target_name)
                    continue

                self.point_target_pubs[target_name].publish(point_target)

                self.publish_marker(
                    marker_id=cfg['marker_id'],
                    point_msg=point_target,
                    color_rgb=cfg['marker_color']
                )

                self.get_logger().info(
                    f'target={target_name}, '
                    f'visible=True, '
                    f'pixel=({result["u"]},{result["v"]}), '
                    f'depth_valid=True, '
                    f'camera_xyz=({point_camera.point.x:.3f}, '
                    f'{point_camera.point.y:.3f}, '
                    f'{point_camera.point.z:.3f}), '
                    f'{self.target_frame}_xyz=({point_target.point.x:.3f}, '
                    f'{point_target.point.y:.3f}, '
                    f'{point_target.point.z:.3f})'
                )

                # 单目标模式下，统一输出
                if self.target_name != 'all':
                    current_target_found = True
                    self.publish_current_visible(True)
                    self.publish_current_target_name_echo(target_name)
                    self.current_point_world_pub.publish(point_target)

                    pregrasp_point = self.make_pregrasp_point(point_target)
                    self.current_pregrasp_world_pub.publish(pregrasp_point)

            except TransformException as e:
                self.delete_marker(cfg['marker_id'])
                self.get_logger().warn(
                    f'target={target_name}, visible=True, '
                    f'camera_xyz=({point_camera.point.x:.3f}, '
                    f'{point_camera.point.y:.3f}, '
                    f'{point_camera.point.z:.3f}), '
                    f'but transform failed: {camera_frame} -> {self.target_frame}, '
                    f'reason={str(e)}'
                )

                if self.target_name != 'all':
                    self.publish_current_visible(False)
                    self.publish_current_target_name_echo(target_name)

        # 单目标模式下，如果整轮都没找到目标，也保证发 False
        if self.target_name != 'all' and not current_target_found:
            self.publish_current_visible(False)
            self.publish_current_target_name_echo(self.target_name)

    # --------------------------------------------------
    # Detection helpers
    # --------------------------------------------------
    def detect_one_target(self, hsv, depth_m, camera_frame, cfg):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in cfg['hsv_ranges']:
            lower_np = np.array(lower, dtype=np.uint8)
            upper_np = np.array(upper, dtype=np.uint8)
            mask_part = cv2.inRange(hsv, lower_np, upper_np)
            mask = cv2.bitwise_or(mask, mask_part)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area < cfg['min_area']:
            return None

        m = cv2.moments(largest_contour)
        if abs(m['m00']) < 1e-6:
            return None

        u = int(m['m10'] / m['m00'])
        v = int(m['m01'] / m['m00'])

        depth_valid, z = self.get_valid_depth(depth_m, mask, u, v)
        if not depth_valid:
            return None

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        point_camera = PointStamped()
        point_camera.header.stamp = self.rgb_msg.header.stamp
        point_camera.header.frame_id = camera_frame
        point_camera.point.x = float(x)
        point_camera.point.y = float(y)
        point_camera.point.z = float(z)

        return {
            'u': u,
            'v': v,
            'point_camera': point_camera,
        }

    def point_in_bin_a_workspace(self, point_msg: PointStamped) -> bool:
        p = point_msg.point
        return (
            self.bin_a_workspace_min[0] <= p.x <= self.bin_a_workspace_max[0]
            and self.bin_a_workspace_min[1] <= p.y <= self.bin_a_workspace_max[1]
            and self.bin_a_workspace_min[2] <= p.z <= self.bin_a_workspace_max[2]
        )

    def make_pregrasp_point(self, point_msg: PointStamped):
        pregrasp = PointStamped()
        pregrasp.header = point_msg.header
        pregrasp.point.x = point_msg.point.x
        pregrasp.point.y = point_msg.point.y
        pregrasp.point.z = point_msg.point.z + self.pregrasp_height
        return pregrasp

    # --------------------------------------------------
    # Publish helpers
    # --------------------------------------------------
    def publish_current_visible(self, visible: bool):
        msg = Bool()
        msg.data = visible
        self.current_visible_pub.publish(msg)

    def publish_current_target_name_echo(self, target_name: str):
        msg = String()
        msg.data = target_name
        self.current_target_name_pub.publish(msg)

    def publish_marker(self, marker_id, point_msg: PointStamped, color_rgb):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = point_msg.header.frame_id
        marker.ns = 'target_marker'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point_msg.point.x
        marker.pose.position.y = point_msg.point.y
        marker.pose.position.z = point_msg.point.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.a = 1.0
        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])

        self.marker_pub.publish(marker)

    def delete_marker(self, marker_id):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.target_frame
        marker.ns = 'target_marker'
        marker.id = marker_id
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    # --------------------------------------------------
    # Utility
    # --------------------------------------------------
    def convert_depth_to_meters(self, depth_image):
        if depth_image.dtype == np.float32:
            return depth_image
        if depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) / 1000.0
        return None

    def get_valid_depth(self, depth_m, mask, u, v):
        h, w = depth_m.shape[:2]

        if 0 <= v < h and 0 <= u < w:
            z = float(depth_m[v, u])
            if math.isfinite(z) and 0.05 < z < 10.0:
                return True, z

        valid = (
            (mask > 0) &
            np.isfinite(depth_m) &
            (depth_m > 0.05) &
            (depth_m < 10.0)
        )

        valid_depths = depth_m[valid]
        if valid_depths.size == 0:
            return False, None

        z = float(np.median(valid_depths))
        return True, z


def main(args=None):
    rclpy.init(args=args)
    node = ColorPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for cfg in node.target_configs.values():
            node.delete_marker(cfg['marker_id'])
        node.destroy_node()
        rclpy.shutdown()
