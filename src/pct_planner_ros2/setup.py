from setuptools import setup
import os
from glob import glob

package_name = 'pct_planner_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        # 添加launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer Name',
    maintainer_email='maintainer@example.com',
    description='PCT Planner for ROS2 Humble',
    license='GPLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pct_planner_node = pct_planner_ros2.plan:main',
        ],
    },
)