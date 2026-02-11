from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'guide_robot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), (glob('launch/*.launch.py'))),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iwintern',
    maintainer_email='cyx2582205445@gmail.com',
    description='Package for guide robot localization',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_cloud_transformer = guide_robot_localization.point_cloud_transformer:main',
            'goal_pose_remapper = guide_robot_localization.goal_pose_remapper:main', 
            'static_odom_publisher = guide_robot_localization.static_odom_publisher:main', 
            'pc_synced_static_odom_publisher = guide_robot_localization.pc_synced_static_odom_publisher:main', 
            'tf_height_remover = guide_robot_localization.tf_height_remover:main', 
        ],
    },
)
