from setuptools import setup

package_name = 'robowheels'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/motor.yaml']),
        ('share/' + package_name + '/launch', ['launch/motor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wu Ruihao',
    maintainer_email='wuruihao629@outlook.com',
    description='RoboWheels differential-drive base controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_driver = robowheels.motor_driver:main',
        ],
    },
)
