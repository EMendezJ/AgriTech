from setuptools import setup

package_name = 'sensor_logger'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests', 'opencv-python'],
    zip_safe=True,
    maintainer='AgriTech Team',
    maintainer_email='user@example.com',
    description='AgriTech sensor logging and control nodes',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'sensor_logger = sensor_logger.sensor_logger_node:main',
            'watering = sensor_logger.watering_node:main',
        ],
    },
)
