#!/usr/bin/env python3
"""
自动驾驶感知模块（精简版·优化·仅点云输出）
功能：对输入点云进行体素降采样、后方地面过滤、聚类，输出障碍物点云。
订阅：/merged_cloud (sensor_msgs/PointCloud2)
发布：
  /ogmsg_points        - 障碍物点云（perception_msgs/OgmPoints）  [下游，可选发布]
  /og_points           - 障碍物点云（sensor_msgs/PointCloud2，红色）[调试，始终发布]
优化点：
  - 增加体素降采样
  - 优先使用 DBSCAN 聚类
  - 移除文本和 MarkerArray
  - 增加后方地面过滤，适应 Ariy 雷达畸变
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from perception_msgs.msg import OgmPoints, OgmPoint

try:
    from scipy.spatial import cKDTree
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

try:
    from sklearn.cluster import DBSCAN
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


# ============================== 辅助函数 ==============================

def extract_xyz(cloud):
    """从 PointCloud2 提取 xyz，跳过无效点。"""
    n = cloud.width * cloud.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)

    offsets = {}
    for field in cloud.fields:
        if field.name in ('x', 'y', 'z'):
            offsets[field.name] = field.offset
    if len(offsets) != 3:
        return np.empty((0, 3), dtype=np.float32)

    raw = np.frombuffer(cloud.data, dtype=np.uint8).reshape(n, cloud.point_step)

    x_bytes = raw[:, offsets['x']:offsets['x']+4]
    y_bytes = raw[:, offsets['y']:offsets['y']+4]
    z_bytes = raw[:, offsets['z']:offsets['z']+4]

    x = x_bytes.view('<f4').reshape(n)
    y = y_bytes.view('<f4').reshape(n)
    z = z_bytes.view('<f4').reshape(n)
    xyz = np.column_stack([x, y, z])

    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz


def voxel_downsample(points, leaf_size):
    """体素降采样，返回保留点索引。"""
    if leaf_size <= 0 or points.shape[0] == 0:
        return np.arange(points.shape[0])
    voxel_indices = np.floor(points / leaf_size).astype(np.int64)
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    return np.sort(unique_indices)


def create_colored_cloud_msg(points, frame_id, stamp, color=(255, 0, 0)):
    """生成带 RGB 的 PointCloud2 消息。"""
    n = points.shape[0]
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
        PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
        PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
    ]
    point_step = 16
    dtype = np.dtype({
        'names': ['x', 'y', 'z', 'r', 'g', 'b'],
        'formats': ['<f4', '<f4', '<f4', 'u1', 'u1', 'u1'],
        'offsets': [0, 4, 8, 12, 13, 14],
        'itemsize': 16
    })
    arr = np.zeros(n, dtype=dtype)
    arr['x'] = points[:, 0].astype(np.float32)
    arr['y'] = points[:, 1].astype(np.float32)
    arr['z'] = points[:, 2].astype(np.float32)
    arr['r'] = color[0]
    arr['g'] = color[1]
    arr['b'] = color[2]
    data_bytes = arr.tobytes()

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = point_step
    msg.row_step = point_step * n
    msg.fields = fields
    msg.data = data_bytes
    return msg


# ============================== 聚类模块（保持不变） ==============================

class ClusterDetector:
    """聚类模块，同前。"""
    def __init__(self, method='auto', tolerance=0.5, min_size=10, max_size=100000):
        self.method = method
        self.tolerance = tolerance
        self.min_size = min_size
        self.max_size = max_size

        if self.method == 'auto':
            self.effective_method = 'dbscan' if SKLEARN_AVAILABLE else 'euclidean'
        else:
            self.effective_method = self.method

    def _euclidean_cluster(self, points):
        if points.shape[0] == 0:
            return []
        if not SCIPY_AVAILABLE:
            return self._bruteforce_cluster(points)
        tree = cKDTree(points)
        pairs = tree.query_pairs(r=self.tolerance, output_type='ndarray')
        n = points.shape[0]
        parent = np.arange(n)

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i, j):
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[ri] = rj

        for i, j in pairs:
            union(i, j)

        clusters_dict = {}
        for i in range(n):
            root = find(i)
            clusters_dict.setdefault(root, []).append(i)
        return list(clusters_dict.values())

    def _bruteforce_cluster(self, points):
        n = points.shape[0]
        visited = np.zeros(n, dtype=bool)
        clusters = []
        for i in range(n):
            if visited[i]:
                continue
            component = [i]
            visited[i] = True
            changed = True
            while changed:
                changed = False
                for j in range(n):
                    if not visited[j]:
                        dist = np.linalg.norm(points[j] - points[component[-1]])
                        if dist < self.tolerance:
                            visited[j] = True
                            component.append(j)
                            changed = True
            clusters.append(component)
        return clusters

    def _dbscan_cluster(self, points):
        if not SKLEARN_AVAILABLE:
            raise ImportError("sklearn not available, cannot use DBSCAN")
        db = DBSCAN(eps=self.tolerance, min_samples=self.min_size).fit(points)
        labels = db.labels_
        clusters = []
        for label in np.unique(labels):
            if label == -1:
                continue
            clusters.append(np.where(labels == label)[0].tolist())
        return clusters

    def cluster(self, points):
        if points.shape[0] == 0:
            return []
        if self.effective_method == 'euclidean':
            raw_clusters = self._euclidean_cluster(points)
        elif self.effective_method == 'dbscan':
            raw_clusters = self._dbscan_cluster(points)
        else:
            raise ValueError(f"Unknown cluster method: {self.effective_method}")
        valid_clusters = []
        for cluster in raw_clusters:
            if self.min_size <= len(cluster) <= self.max_size:
                valid_clusters.append(cluster)
        return valid_clusters


# ============================== 后方地面过滤模块 ==============================

class RearGroundFilter:
    """
    针对车辆后方（Ariy 雷达）地面畸变的地面过滤。
    方法：将后方区域按 x 分条带，在每个条带内对 (y,z) 直线拟合，剔除靠近直线的地面点。
    """
    def __init__(self, min_x=-20.0, max_x=-1.8, bin_size=1.0,
                 min_points_per_bin=5, distance_threshold=0.15):
        self.min_x = min_x
        self.max_x = max_x
        self.bin_size = bin_size
        self.min_points_per_bin = min_points_per_bin
        self.distance_threshold = distance_threshold

    def filter(self, points):
        """
        输入：所有点（体素降采样后） Nx3
        输出：过滤后的非地面点（包括前方点和后方未被剔除的点）
        """
        # 分离后方区域和其他区域
        mask_rear = (points[:, 0] >= self.min_x) & (points[:, 0] <= self.max_x)
        rear_points = points[mask_rear]
        other_points = points[~mask_rear]

        if rear_points.shape[0] == 0:
            return points  # 没有后方点，直接返回

        # 按 x 分条带
        x_min = np.min(rear_points[:, 0])
        x_max = np.max(rear_points[:, 0])
        bins = np.arange(x_min, x_max + self.bin_size, self.bin_size)

        # 存储后方点中保留的索引（相对于 rear_points）
        keep_indices = np.ones(rear_points.shape[0], dtype=bool)

        for i in range(len(bins) - 1):
            bin_mask = (rear_points[:, 0] >= bins[i]) & (rear_points[:, 0] < bins[i+1])
            bin_points = rear_points[bin_mask]
            if bin_points.shape[0] < self.min_points_per_bin:
                continue  # 点数太少，不处理

            y = bin_points[:, 1]
            z = bin_points[:, 2]

            # 最小二乘拟合 z = a*y + b
            A = np.vstack([y, np.ones_like(y)]).T
            # 求解 A * [a, b]^T = z
            try:
                coeffs, _, _, _ = np.linalg.lstsq(A, z, rcond=None)
                a, b = coeffs
            except np.linalg.LinAlgError:
                continue

            # 计算点到直线的垂直距离
            distances = np.abs(z - (a * y + b)) / np.sqrt(a**2 + 1)

            # 距离小于阈值的点认为是地面，需要剔除
            ground_mask = distances < self.distance_threshold

            # 更新保留标志（在当前 bin 内，只保留非地面点）
            # 注意：bin_mask 是相对于 rear_points 的布尔数组
            # 需要将 ground_mask 映射回 rear_points 的索引
            # 使用 np.where 获取当前 bin 在 rear_points 中的位置
            bin_indices = np.where(bin_mask)[0]
            # 对于 bin 内的每个点，如果属于地面，则在 keep_indices 中标记为 False
            for idx_in_bin, is_ground in enumerate(ground_mask):
                if is_ground:
                    keep_indices[bin_indices[idx_in_bin]] = False

        # 得到过滤后的后方点
        rear_filtered = rear_points[keep_indices]

        # 合并前方点和过滤后的后方点
        filtered_points = np.vstack([other_points, rear_filtered]) if other_points.shape[0] > 0 else rear_filtered
        return filtered_points


# ============================== 障碍物发布模块（不变） ==============================

class ObstaclePublisher:
    """发布障碍物点云。"""
    def __init__(self, node, enable_ogmsg=False):
        self.node = node
        if enable_ogmsg:
            self.ogmsg_points_pub = node.create_publisher(OgmPoints, '/ogmsg_points', 10)
        else:
            self.ogmsg_points_pub = None
        self.debug_cloud_pub = node.create_publisher(PointCloud2, '/og_points', 10)

    def publish(self, clusters, points, frame_id, stamp):
        if clusters:
            all_indices = np.concatenate(clusters).astype(int)
        else:
            all_indices = np.array([], dtype=int)

        obstacle_points = points[all_indices]

        # 发布下游点云（可选）
        if self.ogmsg_points_pub is not None:
            og_msg = OgmPoints()
            og_msg.header.frame_id = frame_id
            og_msg.header.stamp = stamp
            for pt in obstacle_points:
                p = OgmPoint()
                p.x = float(pt[0])
                p.y = float(pt[1])
                p.z = float(pt[2])
                og_msg.points.append(p)
            self.ogmsg_points_pub.publish(og_msg)

        # 发布调试红色点云
        if obstacle_points.shape[0] > 0:
            debug_msg = create_colored_cloud_msg(obstacle_points, frame_id, stamp, color=(255, 0, 0))
        else:
            debug_msg = create_colored_cloud_msg(np.empty((0, 3), dtype=np.float32), frame_id, stamp)
        self.debug_cloud_pub.publish(debug_msg)


# ============================== 主节点 ==============================

class PerceptionNode(Node):
    """感知节点：体素降采样 + 后方地面过滤 + 聚类 + 输出"""

    DEFAULT_PARAMS = {
        'cluster_method': 'auto',
        'cluster_tolerance': 0.5,
        'min_cluster_size': 10,
        'max_cluster_size': 100000,
        'voxel_size': 0.1,
        'publish_ogmsg_points': False,
        # 后方地面过滤参数
        'enable_rear_ground_filter': True,
        'rear_min_x': -20.0,
        'rear_max_x': -1.8,
        'rear_bin_size': 1.0,
        'rear_min_points_per_bin': 5,
        'rear_ground_distance_threshold': 0.15,
    }

    def __init__(self):
        super().__init__('perception_node')
        for name, default in self.DEFAULT_PARAMS.items():
            self.declare_parameter(name, default)
        self._load_params()

        self.cluster_detector = ClusterDetector(
            method=self.cluster_method,
            tolerance=self.cluster_tolerance,
            min_size=self.min_cluster_size,
            max_size=self.max_cluster_size
        )
        self.obstacle_publisher = ObstaclePublisher(self, enable_ogmsg=self.publish_ogmsg_points)

        # 如果启用后方地面过滤，创建实例
        if self.enable_rear_ground_filter:
            self.rear_ground_filter = RearGroundFilter(
                min_x=self.rear_min_x,
                max_x=self.rear_max_x,
                bin_size=self.rear_bin_size,
                min_points_per_bin=self.rear_min_points_per_bin,
                distance_threshold=self.rear_ground_distance_threshold
            )
        else:
            self.rear_ground_filter = None

        self.subscription = self.create_subscription(
            PointCloud2, '/merged_cloud', self.cloud_callback, 10)

        self._last_log_time = time.perf_counter()
        self._frame_count = 0
        self._last_process_time = 0.0

        self.get_logger().info(
            f"Perception node started. Cluster: {self.cluster_detector.effective_method}, "
            f"voxel: {self.voxel_size}, rear_filter: {self.enable_rear_ground_filter}"
        )

    def _load_params(self):
        self.cluster_method = self.get_parameter('cluster_method').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.publish_ogmsg_points = self.get_parameter('publish_ogmsg_points').value
        self.enable_rear_ground_filter = self.get_parameter('enable_rear_ground_filter').value
        self.rear_min_x = self.get_parameter('rear_min_x').value
        self.rear_max_x = self.get_parameter('rear_max_x').value
        self.rear_bin_size = self.get_parameter('rear_bin_size').value
        self.rear_min_points_per_bin = self.get_parameter('rear_min_points_per_bin').value
        self.rear_ground_distance_threshold = self.get_parameter('rear_ground_distance_threshold').value

    def cloud_callback(self, msg):
        t_start = time.perf_counter()

        points = extract_xyz(msg)
        if points.shape[0] == 0:
            self.get_logger().warning("Received empty point cloud", throttle_duration_sec=5.0)
            return

        if self.voxel_size > 0:
            idx = voxel_downsample(points, self.voxel_size)
            points = points[idx]

        # 应用后方地面过滤
        if self.rear_ground_filter is not None:
            points = self.rear_ground_filter.filter(points)

        clusters = self.cluster_detector.cluster(points)
        self.obstacle_publisher.publish(clusters, points, msg.header.frame_id, msg.header.stamp)

        t_end = time.perf_counter()
        self._last_process_time = (t_end - t_start) * 1000.0
        self._frame_count += 1
        if t_end - self._last_log_time >= 5.0:
            self.get_logger().info(
                f"[perception] {self._frame_count} frames/5s, "
                f"points after filter: {points.shape[0]}, clusters: {len(clusters)}, "
                f"{self._last_process_time:.1f} ms/frame"
            )
            self._last_log_time = t_end
            self._frame_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    