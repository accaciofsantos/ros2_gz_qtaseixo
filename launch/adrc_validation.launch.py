from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    world_path = os.path.expanduser('~/ros2_ws/src/ros2_gz_qtaseixo/worlds/variable_friction_world.world')

    return LaunchDescription([
        # Inicia Gazebo com o mundo com solo variavel
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', world_path],
            output='screen'
        ),

        # Spawna o robô terrestre (UGV)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'ugv',
                '-x', '0', '-y', '0', '-z', '0.5',
                '-file', os.path.expanduser('~/ros2_ws/src/ros2_gz_qtaseixo/models/COSTAR_HUSKY_SENSOR_CONFIG_2/model.sdf')
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
