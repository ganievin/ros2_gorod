from setuptools import setup

package_name = 'city_graph'

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
    description='Graph storage and goal management for city hackathon',
    license='MIT',
    entry_points={
        'console_scripts': [
            'graph_storage_node = city_graph.graph_storage_node:main',
        ],
    },
)

