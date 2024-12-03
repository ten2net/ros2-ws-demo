from setuptools import find_packages, setup
import os

package_name = 'entity_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml','launch/turtlesim.launch.py']),
        # ('share/' + package_name, ['package.xml',glob('launch/*.launch.py')] ),
        # (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ten2net@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_node = entity_controller.TurtleNode:main',
            'sub_warship = entity_controller.WarShipSubscriber:main',
            'pub_warship = entity_controller.WarShipPublisher:main',
            'WarShip = entity_controller.WarShip:main'
        ],
    },
)
