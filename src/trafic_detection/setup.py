from glob import glob
import os

from setuptools import setup

package_name = 'trafic_detection'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='yana',
    maintainer_email='yana@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = trafic_detection.detector_node:main',
            'distance_estimator = trafic_detection.distance_estimator:main',
            'sign_handler_node = trafic_detection.sign_handler_node:main',
            'lf_node = trafic_detection.lf_node:main',
        ],
    },
)
