from setuptools import setup

package_name = 'puzzlebot_linefollower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Line follower for Manchester Puzzlebot using OpenCV and ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'line_follower = puzzlebot_linefollower.line_follower:main'
        ],
    },
)

