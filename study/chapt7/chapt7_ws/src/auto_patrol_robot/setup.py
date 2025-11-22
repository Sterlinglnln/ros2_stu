from setuptools import find_packages, setup

package_name = 'auto_patrol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/config", ['config/patrol_config.yaml']),
        ('share/' + package_name+"/launch", ['launch/autopatrol.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larry',
    maintainer_email='fpnf97@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'patrol_node = auto_patrol_robot.patrol_node:main',
            'speaker = auto_patrol_robot.speaker:main',
        ],
    },
)
