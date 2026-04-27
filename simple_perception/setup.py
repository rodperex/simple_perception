from setuptools import setup
import os
from glob import glob

package_name = 'simple_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo Pérez-Rodríguez',
    maintainer_email='rodrigo.perez@urjc.es',
    description='Simple perception utilities for object tracking and detection conversion',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'entity_tracker = simple_perception.entity_tracker:main',
            'entity_tracker_fake_3d = simple_perception.entity_tracker_fake_3d:main',
            'entity_tracker_monocular_3d = simple_perception.entity_tracker_monocular_3d:main',
            'depth_calibration_helper = simple_perception.depth_calibration_helper:main',
            'yolo_to_standard = simple_perception.yolo_to_standard:main',
        ],
    },
)
