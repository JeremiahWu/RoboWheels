from setuptools import setup

package_name = 'robowheels'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
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
        ('share/' + package_name + '/config/rviz', [
            'config/rviz/perception.rviz',
            'config/rviz/rdk_debug.rviz',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/motor.launch.py',
            'launch/joy.launch.py',
            'launch/teleop.launch.py',
            'launch/astra_camera.launch.py',
            'launch/gs130.launch.py',
            'launch/stereo_pointcloud.launch.py',
            'launch/bringup.launch.py',
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
    description='RoboWheels differential-drive base controller (RDK X5)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_driver = robowheels.motor_driver:main',
            'astra_rgb_node = robowheels.astra_rgb_node:main',
            'gs130_vio_bridge = robowheels.gs130_vio_bridge:main',
            'stereo_pointcloud_node = robowheels.stereo_pointcloud_node:main',
        ],
    },
)
