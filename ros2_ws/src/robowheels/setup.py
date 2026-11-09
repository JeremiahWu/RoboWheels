from setuptools import find_packages, setup

package_name = 'robowheels'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/motor.yaml',
            'config/joy.yaml',
            'config/astra_rgb.yaml',
            'config/gs130_vio.yaml',
            'config/stereo_pc.yaml',
        ]),
        ('share/' + package_name + '/config/arms', [
            'config/arms/servo_bridge.example.json',
            'config/arms/servo_zero_offsets.example.json',
        ]),
        ('share/' + package_name + '/config/rviz', [
            'config/rviz/perception.rviz',
            'config/rviz/rdk_debug.rviz',
            'config/rviz/mac_astra.rviz',
            'config/rviz/physical_arms.rviz',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/motor.launch.py',
            'launch/joy.launch.py',
            'launch/teleop.launch.py',
            'launch/astra_camera.launch.py',
            'launch/gs130.launch.py',
            'launch/stereo_pointcloud.launch.py',
            'launch/bringup.launch.py',
            'launch/navigation_preview.launch.py',
            'launch/navigation_bpu.launch.py',
            'launch/vio.launch.py',
            'launch/rviz_perception.launch.py',
        ]),
        ('share/' + package_name + '/udev', [
            'udev/99-robowheels.rules',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wu Ruihao',
    maintainer_email='wuruihao629@outlook.com',
    description='RoboWheels differential base, stereo navigation, bus-servo arms and Astra sensors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'servo_feedback_bridge = robowheels.arms.ros_shadow:main',
            'mac_servo_bridge = robowheels.arms.mac_bridge:main',
            'host_image_receiver = robowheels.sensors.host_image_receiver:main',
            'mac_astra_capture = robowheels.sensors.mac_astra_capture:main',
            'stereonet_adapter = robowheels.perception.stereonet_adapter:main',
            'orb_odometry_node = robowheels.localization.orb_odometry_node:main',
            'local_map_node = robowheels.perception.local_map_node:main',
            'planning_node = robowheels.planning.planning_node:main',
            'motor_driver = robowheels.base.motor_driver:main',
            'astra_rgb_node = robowheels.sensors.astra_rgb_node:main',
            'gs130_vio_bridge = robowheels.sensors.gs130_vio_bridge:main',
            'stereo_pointcloud_node = robowheels.perception.stereo_pointcloud_node:main',
        ],
    },
)
