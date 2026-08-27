#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener
from message_filters import ApproximateTimeSynchronizer, Subscriber

# 输出点云布局：xyz(float32) + rgb(uint8)，点步长 16 字节
_OUT_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('r', 'u1'), ('g', 'u1'), ('b', 'u1'),
])
_POINT_STEP = _OUT_DTYPE.itemsize  # 16


def quaternion_to_rotation_matrix(q):
    """将四元数 (x, y, z, w) 转换为 3x3 旋转矩阵。"""
    x, y, z, w = q
    return np.array([
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y]
    ])


def extract_xyz(cloud):
    """从 PointCloud2 向量化提取 xyz 为 Nx3 float32（跳过 NaN）。"""
    n = cloud.width * cloud.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)

    off = {}
    for f in cloud.fields:
        if f.name in ('x', 'y', 'z'):
            off[f.name] = f.offset
    if len(off) != 3:
        return np.empty((0, 3), dtype=np.float32)

    raw = np.frombuffer(cloud.data, dtype=np.uint8).reshape(n, cloud.point_step)

    def col(o):
        return raw[:, o:o + 4].copy().view('<f4').reshape(n)

    xyz = np.column_stack([col(off['x']), col(off['y']), col(off['z'])])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz


def voxel_downsample(points, leaf):
    """体素降采样，返回保留点索引（保持原顺序）。"""
    vox = np.floor(points / leaf).astype(np.int64)
    _, idx = np.unique(vox, axis=0, return_index=True)
    return np.sort(idx)


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')

        # 两个输入话题：左 MID360 和速腾 Arya
        self.left_topic = '/livox/lidar'
        self.rslidar_topic = '/rslidar_points'

        # 创建两个订阅器（用于同步）
        self.sub_left = Subscriber(self, PointCloud2, self.left_topic)
        self.sub_rslidar = Subscriber(self, PointCloud2, self.rslidar_topic)

        # 时间同步（允许 0.5 秒的误差）
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_rslidar],
            queue_size=10,
            slop=0.5
        )
        self.sync.registerCallback(self.merge_and_publish)

        # 发布融合后的点云
        self.pub = self.create_publisher(PointCloud2, '/merged_cloud', 10)

        # ==================== 参数声明 ====================
        # 目标坐标系和体素大小
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('voxel_size', 0.05)

        # ROI 滤波（保留区域）
        self.declare_parameter('filter_enable_roi', False)
        self.declare_parameter('roi_min_x', -10.0)
        self.declare_parameter('roi_max_x', 20.0)
        self.declare_parameter('roi_min_y', -4.0)
        self.declare_parameter('roi_max_y', 4.0)
        self.declare_parameter('roi_min_z', -0.5)
        self.declare_parameter('roi_max_z', 3.0)

        # ===== 新增：排除区域（剔除自身包围盒） =====
        self.declare_parameter('filter_enable_exclude', False)
        self.declare_parameter('exclude_min_x', -2.0)
        self.declare_parameter('exclude_max_x', 0.65)
        self.declare_parameter('exclude_min_y', -0.85)
        self.declare_parameter('exclude_max_y', 0.85)
        self.declare_parameter('exclude_min_z', 0.0)
        self.declare_parameter('exclude_max_z', 3.0)

        # 地面分割（阈值）
        self.declare_parameter('filter_enable_ground', False)
        self.declare_parameter('ground_height_threshold', 0.1)

        # 读取参数
        self.target_frame = self.get_parameter('target_frame').value
        self.voxel_size = self.get_parameter('voxel_size').value

        # ROI
        self.filter_enable_roi = self.get_parameter('filter_enable_roi').value
        self.roi_min_x = self.get_parameter('roi_min_x').value
        self.roi_max_x = self.get_parameter('roi_max_x').value
        self.roi_min_y = self.get_parameter('roi_min_y').value
        self.roi_max_y = self.get_parameter('roi_max_y').value
        self.roi_min_z = self.get_parameter('roi_min_z').value
        self.roi_max_z = self.get_parameter('roi_max_z').value

        # 排除区域
        self.filter_enable_exclude = self.get_parameter('filter_enable_exclude').value
        self.exclude_min_x = self.get_parameter('exclude_min_x').value
        self.exclude_max_x = self.get_parameter('exclude_max_x').value
        self.exclude_min_y = self.get_parameter('exclude_min_y').value
        self.exclude_max_y = self.get_parameter('exclude_max_y').value
        self.exclude_min_z = self.get_parameter('exclude_min_z').value
        self.exclude_max_z = self.get_parameter('exclude_max_z').value

        # 地面
        self.filter_enable_ground = self.get_parameter('filter_enable_ground').value
        self.ground_height_threshold = self.get_parameter('ground_height_threshold').value

        # TF 监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF 缓存（减少重复查询）
        self._tf_cache = {}

        # 统计信息
        self._log_t0 = time.perf_counter()
        self._log_frames = 0
        self._last_ms = 0.0

        self.get_logger().info(
            f'PointCloud Merger Node Started (2 inputs: {self.left_topic}, {self.rslidar_topic})'
        )

    def get_transform(self, frame_id):
        """获取 frame_id 到 target_frame 的变换，带 5 秒缓存。"""
        now = self.get_clock().now()
        cached = self._tf_cache.get(frame_id)
        if cached is not None and (now - cached['t']).nanoseconds < 5e9:
            return cached['tf']
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, rclpy.time.Time())
        except Exception as e:
            if cached is not None:
                self.get_logger().warn(f'TF lookup failed, using cached for {frame_id}', throttle_duration_sec=5.0)
                return cached['tf']
            raise
        self._tf_cache[frame_id] = {'t': now, 'tf': tf}
        return tf

    def transform_points(self, points, transform):
        """将点云 (Nx3) 根据 transform 旋转并平移。"""
        t = transform.transform.translation
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        q = transform.transform.rotation
        rot = quaternion_to_rotation_matrix([q.x, q.y, q.z, q.w]).astype(np.float32)
        return (rot @ points.T).T + translation

    def apply_filters(self, points, colors):
        """
        按顺序应用 ROI 保留、排除区域剔除、地面过滤。
        返回过滤后的点云和颜色数组。
        """
        if points.shape[0] == 0:
            return points, colors

        mask = np.ones(points.shape[0], dtype=bool)

        # 1. ROI 保留（如果启用）
        if self.filter_enable_roi:
            mask &= (points[:, 0] >= self.roi_min_x) & (points[:, 0] <= self.roi_max_x) \
                    & (points[:, 1] >= self.roi_min_y) & (points[:, 1] <= self.roi_max_y) \
                    & (points[:, 2] >= self.roi_min_z) & (points[:, 2] <= self.roi_max_z)

        # 2. 排除区域剔除（如果启用）—— 剔除落在该长方体内的点
        if self.filter_enable_exclude:
            exclude_mask = (points[:, 0] >= self.exclude_min_x) & (points[:, 0] <= self.exclude_max_x) \
                         & (points[:, 1] >= self.exclude_min_y) & (points[:, 1] <= self.exclude_max_y) \
                         & (points[:, 2] >= self.exclude_min_z) & (points[:, 2] <= self.exclude_max_z)
            mask &= ~exclude_mask

        # 3. 地面分割（保留高于阈值的点）
        if self.filter_enable_ground:
            mask &= (points[:, 2] > self.ground_height_threshold)

        return points[mask], colors[mask]

    def merge_and_publish(self, cloud_left, cloud_rs):
        t0 = time.perf_counter()
        try:
            # 获取各雷达的 TF
            tf_left = self.get_transform(cloud_left.header.frame_id)
            tf_rs = self.get_transform(cloud_rs.header.frame_id)

            # 提取点云
            pts_left = extract_xyz(cloud_left)
            pts_rs = extract_xyz(cloud_rs)

            if pts_left.shape[0] == 0 and pts_rs.shape[0] == 0:
                return

            # 变换到目标坐标系
            if pts_left.shape[0] > 0:
                pts_left = self.transform_points(pts_left, tf_left)
            if pts_rs.shape[0] > 0:
                pts_rs = self.transform_points(pts_rs, tf_rs)

            # 合并
            merged = np.vstack([pts for pts in (pts_left, pts_rs) if pts.shape[0] > 0])

            # 根据来源分配颜色（左-红，速腾-蓝）
            n_left = pts_left.shape[0]
            n_rs = pts_rs.shape[0]
            total = n_left + n_rs
            colors = np.zeros((total, 3), dtype=np.uint8)
            if n_left > 0:
                colors[:n_left, 0] = 255       # 红
            if n_rs > 0:
                colors[n_left:, 2] = 255          # 蓝

            # ===== 数据过滤（ROI保留 → 排除剔除 → 地面分割） =====
            if merged.shape[0] > 0:
                merged, colors = self.apply_filters(merged, colors)

            # 体素降采样（如果 v > 0）
            if self.voxel_size > 0 and merged.shape[0] > 0:
                idx = voxel_downsample(merged, self.voxel_size)
                merged = merged[idx]
                colors = colors[idx]

            n = merged.shape[0]
            if n == 0:
                return

            # 构建输出点云
            out = np.empty(n, dtype=_OUT_DTYPE)
            out['x'], out['y'], out['z'] = merged[:, 0], merged[:, 1], merged[:, 2]
            out['r'], out['g'], out['b'] = colors[:, 0], colors[:, 1], colors[:, 2]

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
                PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
                PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
            ]

            cloud_msg = PointCloud2()
            cloud_msg.header.frame_id = self.target_frame
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            cloud_msg.height = 1
            cloud_msg.width = n
            cloud_msg.is_bigendian = False
            cloud_msg.is_dense = True
            cloud_msg.point_step = _POINT_STEP
            cloud_msg.row_step = _POINT_STEP * n
            cloud_msg.fields = fields
            cloud_msg.data = out.tobytes()

            self.pub.publish(cloud_msg)
            self._last_ms = (time.perf_counter() - t0) * 1000.0

        except Exception as e:
            self.get_logger().warn(f'Merge failed: {e}', throttle_duration_sec=5.0)
            return

        # 每 5 秒打印一次统计
        self._log_frames += 1
        if time.perf_counter() - self._log_t0 >= 5.0:
            self.get_logger().info(
                f'[merge] {self._log_frames} frames/5s, last {n} pts, {self._last_ms:.1f} ms/frame'
            )
            self._log_t0 = time.perf_counter()
            self._log_frames = 0


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    