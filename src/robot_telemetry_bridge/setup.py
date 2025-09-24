from setuptools import find_packages, setup

package_name = 'robot_telemetry_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tommy',
    maintainer_email='tommyjbrownson@comcast.net',
    description='Parse zmq data and publish to ROS2 topic',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_bridge = robot_telemetry_bridge.node:main',
        ],
    },
)
