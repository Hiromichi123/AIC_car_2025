from setuptools import setup
import os
from glob import glob

package_name = 'car_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ny',
    maintainer_email='ny@todo.todo',
    description='The car_bringup package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odo = car_bringup.odo:main',
            'pubv = car_bringup.pubv:main',
            'newt = car_bringup.newt:main',
        ],
    },
)
