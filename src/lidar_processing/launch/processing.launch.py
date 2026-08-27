from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # ============================================================
        # 1.超级节点：滤波 + 聚类 + 障碍物信息输出
        # ============================================================
        Node(
            package='lidar_processing',      # 请确保与你的 ROS2 包名一致
            executable='super_node',         # 入口点名称，对应 setup.py 里的 console_scripts
            name='super_node',               # 节点运行时名称（可自定义）
            output='screen',                 # 日志输出到终端
            parameters=[
                # ==================== 体素降采样 ====================
                {'voxel_size': 0.1},
                # 体素边长（米），0 表示不降采样

                # ==================== 聚类方法 ====================
                {'cluster_method': 'auto'},
                # 可选 'auto' / 'euclidean' / 'dbscan'

                {'cluster_tolerance': 0.5},
                # 聚类距离阈值（米）

                {'min_cluster_size': 10},
                # 最小聚类点数

                {'max_cluster_size': 100000},
                # 最大聚类点数

                # ==================== 下游点云开关 ====================
                {'publish_ogmsg_points': True},
                # 是否发布下游控制点云 /ogmsg_points（默认可设为 False）
                # 如果设置为 False，则不会创建该发布器，不占用额外 CPU
            ]
        ),
        
       

    ])
    
    
    
    
