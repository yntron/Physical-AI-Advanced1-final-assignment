import os
from glob import glob

from setuptools import setup

package_name = 'pai_safety_stop'


def tree(src):
    """models/ 以下をディレクトリ構造ごと share にインストールする"""
    files = []
    for dirpath, _, filenames in os.walk(src):
        if filenames:
            files.append((
                os.path.join('share', package_name, dirpath),
                [os.path.join(dirpath, f) for f in filenames],
            ))
    return files


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        *tree('models'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='AI safety stop: YOLO person detection brakes a Gazebo robot driven via ROS 2',
    license='AGPL-3.0-only',
    entry_points={
        'console_scripts': [
            'safety_stop = pai_safety_stop.safety_stop_node:main',
            'record_video = pai_safety_stop.record_video:main',
        ],
    },
)
