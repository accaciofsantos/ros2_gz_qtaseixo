from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    #world_path = os.path.expanduser('~/ros2_ws/src/ros2_gz_qtaseixo/worlds/orchard.world')  # clearpath world!   
    #world_path = os.path.expanduser('~/ros2_ws/src/ros2_gz_qtaseixo/worlds/scenario1.world') # dense canopy!
    world_path = os.path.expanduser('~/ros2_ws/src/ros2_gz_qtaseixo/worlds/scenario2_generated.world') # easy world!
    #world_path = os.path.expanduser('~/ros2_ws/src/ros2_gz_qtaseixo/worlds/scenario3.world') # desaligned world!

    return LaunchDescription([
        # Inicia Gazebo com o mundo da quinta do seixo
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', world_path],
            output='screen'
        ),

        # Spawna o robô terrestre (UGV)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'ugv',
                '-x', '-7', '-y', '-6', '-z', '0.5',
                '-file', os.path.expanduser('~/ros2_ws/src/ros2_gz_qtaseixo/models/COSTAR_HUSKY_SENSOR_CONFIG_2/model.sdf')
            ],
            output='screen'
        ),

        # Spawna o UAV
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'uav',
                '-x', '-7', '-y', '-4', '-z', '0.5',
                '-file', os.path.expanduser('~/ros2_ws/src/ros2_gz_qtaseixo/models/X4_GPS_LIDAR_RGBD/model.sdf')
            ],
            output='screen'
        ),

        # Bridge para tópicos do UAV
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/uav/gps@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
                '/uav/air_pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure',
                '/uav/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/uav/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.MagneticField',
                '/uav/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/model/uav/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose',
                '/uav/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                # adicione outros tópicos aqui no mesmo formato
            ],
            output='screen'
        ),

        # Bridge para tópicos do UGV
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/ugv/gps@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
                '/ugv/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/ugv/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/model/ugv/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose',
                '/ugv/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                # outros tópicos UGV
            ],
            output='screen'
        ),
    ])
