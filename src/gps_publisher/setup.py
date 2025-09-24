from setuptools import find_packages, setup

package_name = 'gps_publisher'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Publishes GPS data to /robot/gps/fix',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_publisher = gps_publisher.pub:main',
        ],
    },
)
