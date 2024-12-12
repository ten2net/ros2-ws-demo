from setuptools import find_packages, setup

package_name = 'webots_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/my_robot.urdf']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/my_world.wbt']),
        ('share/' + package_name, ['package.xml']),
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
            'obstacle_avoider = webots_demo.obstacle_avoider:main',
            'my_robot_driver = webots_demo.my_robot_driver:main'
        ],
    },
)
