#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs_py import point_cloud2
import numpy as np

class CustomToPointCloud2(Node):
    def __init__(self):
        super().__init__('custom_to_pointcloud2_py')
        # 订阅 Livox 自定义消息
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.listener_callback,
            10)
        # 发布标准 PointCloud2 消息
        self.publisher = self.create_publisher(PointCloud2, '/livox/pointcloud2', 10)
        self.get_logger().info('CustomToPointCloud2 Python Node Started')

    def listener_callback(self, msg):
        # 1. 从 CustomMsg 中提取有效点的 xyz 坐标
        points = []
        for p in msg.points:
            if p.tag == 0:  # tag 为 0 表示有效点
                points.append([p.x, p.y, p.z])
        
        if not points:
            return

        # 2. 转换为 NumPy 数组
        points_np = np.array(points, dtype=np.float32)

        # 3. 构造 PointCloud2 消息
        header = msg.header
        header.frame_id = "livox_frame"
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = point_cloud2.create_cloud(header, fields, points_np)
        self.publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomToPointCloud2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
