from setuptools import setup

package_name = 'pb2025_nav2_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools', 'pyyaml'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/patrol_launch.py']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
    ],
    zip_safe=True,
    maintainer='PolarBear',
    maintainer_email='you@example.com',
    description='Simple patrol package for nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = pb2025_nav2_patrol.patrol_node:main'
        ],
    },
)
