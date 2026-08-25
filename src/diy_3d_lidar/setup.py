from setuptools import find_packages, setup

package_name = 'diy_3d_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/diy_3d_lidar_launch.py',
            'launch/diy_3d_lidar_rover_launch.py',
            'launch/diy_3d_lidar_laptop_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jun',
    maintainer_email='user@todo.com',
    description='DIY 3D LiDAR with RPLiDAR C1 and tilting stepper motor',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tilt_tf_broadcaster = diy_3d_lidar.tilt_tf_broadcaster:main',
            'scan_to_pointcloud = diy_3d_lidar.scan_to_pointcloud:main',
            'mock_thermal_detector = diy_3d_lidar.mock_thermal_detector:main',
            'multimodal_fusion_node = diy_3d_lidar.multimodal_fusion_node:main',
        ],
    },
)
