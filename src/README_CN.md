# 1 加载工作空间
source /home/pc/why/install/setup.bash

## 1.1 雷达驱动
# rslidar driver
启动雷达
ros2 launch rslidar_sdk start.py
# livox driver
启动雷达
ros2 launch livox_ros_driver2 msg_MID360_launch.py


### 1.1.1 功能包lidar_processing
ros2 run lidar_processing fusion_node   融合点云
ros2 launch lidar_processing drivers.launch.py 	超级节点1
ros2 launch lidar_processing processing.launch.py	超级节点2（1+1+1）
ros2 run lidar_processing super_node 
                                        
重新编译
cd /home/nvidia/ws_tju_test
colcon build --packages-select lidar_processing --symlink-install
colcon build --symlink-install

###雷达硬件ip设置
#速腾Ariy 先通过wireshark抓包 找到活跃ip 然后网页打开该ip  192.168.x.xxx 
#大疆Mid360
直连雷达 → 临时改电脑IP → 创建配置 → 执行工具改IP → 验证 → 恢复网络
1.直连雷达
2.电脑临时 IP （命令行或者图形化设置都可以）
	sudo ip addr del 192.168.8.100/24 dev eno2 2>/dev/null
	sudo ip addr add 192.168.1.50/24 dev eno2
3.验证连通
	ping 192.168.1.180 -c 3
4.进入工具目录
	/home/pc/why/src/driver_livox/Livox-SDK2/build/samples/livox_lidar_ip_set
5.创建 set_ip_config.json
	cat > set_ip_config.json << 'EOF'
	{
	  "lidar_summary_info": {
	    "lidar_type": 8
	  },
	  "MID360": {
	    "lidar_net_info": {
	      "cmd_data_port": 56100,
	      "push_msg_port": 56200,
	      "point_data_port": 56300,
	      "imu_data_port": 56400,
	      "log_data_port": 56500
	    },
	    "host_net_info": [
	      {
		"lidar_ip": ["192.168.1.180"],
		"host_ip": "192.168.1.50",
		"cmd_data_port": 56101,
		"push_msg_port": 56201,
		"point_data_port": 56301,
		"imu_data_port": 56401,
		"log_data_port": 56501
	      }
	    ]
	  },
	  "lidar_configs": [
	    {
	      "ip": "192.168.1.180",
	      "pcl_data_type": 1,
	      "pattern_mode": 0,
	      "extrinsic_parameter": {
		"roll": 0.0,
		"pitch": 0.0,
		"yaw": 0.0,
		"x": 0,
		"y": 0,
		"z": 0
	      }
	    }
	  ]
	}
	EOF
关键字段说明：
lidar_ip：雷达当前 IP → 192.168.1.180
host_ip：电脑当前 IP → 192.168.1.50
ip：雷达当前 IP → 192.168.1.180

6.执行改 IP 命令
./livox_lidar_ip_set set_ip_config.json 192.168.8.180 255.255.255.0 192.168.8.1

参数说明：
192.168.8.180 → 雷达要改成的新 IP
255.255.255.0 → 子网掩码
192.168.8.1 → 网关

7.验证新 IP，恢复电脑 IP 并接入路由器
8.更新驱动配置文件
	gedit /home/pc/why/src/driver_livox/livox_ros_driver2/config/MID360_config.json






