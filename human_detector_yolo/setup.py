import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'human_detector_yolo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*.param') + glob('models/*.bin')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fedde-laptop',
    maintainer_email='feddejorritsma@proton.me',
    description='Human detection using YOLO-FastestV2 on NCNN (CPU-only).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'human_detector_yolo = human_detector_yolo.human_detector_node:main',
        ],
    },
)
