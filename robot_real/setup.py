import os
from glob import glob
from setuptools import setup

package_name = 'robot_real'

setup(
    name=package_name,
    version='1.0.0',
    packages=['scripts'],
    package_dir={'scripts': 'scripts'},
    data_files=[
        # 必要的 ament index 文件
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件目录
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'launch',
        'launch_ros',
        'rclpy',
    ],
    zip_safe=True,
    maintainer='Hiromichi123',
    maintainer_email='2271612727@qq.com',
    description='robot_real',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'robot_real = scripts.main:main',
        ],
    },
)