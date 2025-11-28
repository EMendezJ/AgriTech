from setuptools import find_packages, setup

package_name = 'sensor_logger'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emendezj',
    maintainer_email='emendezj@agritech.com',
    description='AgriTech ROS2 to API bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensor_logger = sensor_logger.sensor_logger_node:main',
        ],
    },
)
