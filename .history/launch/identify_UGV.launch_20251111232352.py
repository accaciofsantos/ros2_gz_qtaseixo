from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def launch_setup(context, *args, **kwargs):
    world_type = LaunchConfiguration('world').perform(context)
    world_dir = os.path.expanduser('~/ros2_ws/src/ros2_gz_qtaseixo/worlds')

    world_map = {
        'firm': f'{world_dir}/scenario_solo_firm.world',
        'medium': f'{world_dir}/scenario_solo_medium.world',
        'slippery': f'{world_dir}/scenario_solo_slippery.world',
        'val_low-': f'{world_dir}/validation_solo_low-.world',
        'val_high': f'{world_dir}/validation_solo_high.world',
        'val_medium': f'{world_dir}/validation_solo_medium.world',
        'val_low': f'{world_dir}/validation_solo_low.world',
    }

    world_file = world_map.get(world_type, world_map['medium'])

    return [
        # Inicia Gazebo com o mundo selecionado
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', world_file],
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
            ],
            output='screen'
        ),
    ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='medium',
        description='Tipo de solo: firm, medium ou slippery'
    )

    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup)
    ])
