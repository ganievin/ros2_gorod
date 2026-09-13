from setuptools import setup

package_name = 'low_lvl_comm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Low-level communication with STM32 over USB-serial',
    license='MIT',
    entry_points={
        'console_scripts': [
            'STM_comm_node = low_lvl_comm.STM_comm_node:main',
        ],
    },
)
