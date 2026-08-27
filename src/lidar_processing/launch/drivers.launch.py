from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ============================================================
        # 1. 启动 Mid-360 驱动
        # ============================================================
        ExecuteProcess(
            cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py'],
            output='screen'
        ),

        # ============================================================
        # 2. 启动速腾 Airy 驱动
        # ============================================================
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rslidar_sdk', 'start.py'],
            output='screen'
        ),
        

	# ============================================================
	# 4. 静态 TF（雷达安装位置）
	# ============================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_livox',
            arguments=['0.5', '0.18', '0', '0', '0', '0', 'base_link', 'livox_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_rslidar',
            arguments=['0', '0', '0.5', '0', '-1.5708', '0', 'base_link', 'rslidar'],
        ),
	    # ============================================================
        # 5. 融合节点
        # ============================================================
        Node(
            package='lidar_processing',
            executable='fusion_node',
            name='pointcloud_merger',
            output='screen',
             parameters=[{
		    # 目标坐标系（TF）
		    'target_frame': 'base_link',

		    # ===== 体素降采样 =====
		    'voxel_size': 0.05,

		    # ===== ROI 滤波（基于 base_link） =====
		    'filter_enable_roi': True,
		    'roi_min_x': -6.0,
		    'roi_max_x':  8.0,
		    'roi_min_y': -4.0,
		    'roi_max_y':  4.0,
		    'roi_min_z': -0.5,
		    'roi_max_z':  2.0,

		    # ===== 排除区域（剔除自身车体） =====
		    'filter_enable_exclude': True,
		    'exclude_min_x': -0.8,
		    'exclude_max_x': 0,
		    'exclude_min_y': -0.2,
		    'exclude_max_y': 0.2,
		    'exclude_min_z': -0.2,
		    'exclude_max_z': 0.1,

		    # ===== 地面分割（高度阈值） =====
		    'filter_enable_ground': True,
		    'ground_height_threshold': 0.1,
		}],
        ),
	
	
	
    ])
    
    
