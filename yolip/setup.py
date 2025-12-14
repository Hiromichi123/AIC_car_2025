from setuptools import setup

package_name = 'yolip'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name + '.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'torch',
        'torchvision',
        'ultralytics',
        'opencv-python',
        'pyyaml',
        'matplotlib',
        'tqdm',
        'numpy',
        'rclpy',
        'cv_bridge',
        'sensor_msgs',
    ],
    zip_safe=True,
    maintainer='Hiromichi123',
    maintainer_email='2271612727@qq.com',
    description='clip + yolo包',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'yolip = yolip.scripts.main:main'
        ],
    },
)
