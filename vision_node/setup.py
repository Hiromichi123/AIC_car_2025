from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_node'

ocr_packages = [
    'ocr',
    'ocr.ppocr',
    'ocr.ppocr.data',
    'ocr.ppocr.losses',
    'ocr.ppocr.metrics',
    'ocr.ppocr.optimizer',
    'ocr.ppocr.postprocess',
    'ocr.ppocr.utils',
    'ocr.tools',
    'ocr.tools.infer',
    # 'ocr.ppocr.utils.e2e_utils',
]

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']) + ocr_packages,
    package_data={
        'ocr.ppocr.utils': ['*.txt', 'dict/*.txt'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装OCR字典文件（但实际使用源码目录的文件）
        ('share/' + package_name + '/ocr/ppocr/utils', 
            glob('ocr/ppocr/utils/*.txt')),
        # 安装YOLO字体文件（但实际使用源码目录的文件）
        ('share/' + package_name + '/yolo',
            (glob('yolo/*.ttf') + glob('yolo/*.pt')) if os.path.exists('yolo') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='dev',
    maintainer_email='fermata@h2o.moe',
    description='vision node for ocr and obj dectection',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "vision_gazebo = vision_node.vision_gazebo:main",
            "vision_real = vision_node.vision_real:main"
        ],
    },
)
