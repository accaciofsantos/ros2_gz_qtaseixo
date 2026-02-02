from setuptools import setup
import os
from glob import glob

package_name = 'ros2_gz_qtaseixo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ADICIONE ESTA LINHA ABAIXO ⬇️
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='accacio@cefetmg.br',
    description='Pacote de controle para do UGV e UAV com ROS2 e Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ugv_teleop_pose  = ros2_gz_qtaseixo.ugv_teleop_pose:main',
            'ugv_control_node = ros2_gz_qtaseixo.ugv_control_node:main',
            'uav_control_node = ros2_gz_qtaseixo.uav_control_node:main',
            'collab_path_node = ros2_gz_qtaseixo.collab_path_node:main',
            'ugv_excitation_gettingData_node = ros2_gz_qtaseixo.ugv_excitation_gettingData_node:main',
            'ardc_controle_node = ardc_controle_node:main',
        ],
    },
)