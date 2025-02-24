from setuptools import find_packages, setup
from glob import glob

package_name = 'cw1_team_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jlcucumber',
    maintainer_email='jasonli1207@foxmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cw1_edge_follower = cw1_team_2.task1.cw1_edge_follower:main',
            'cw1_waypoint_follower = cw1_team_2.task1.cw1_waypoint_follower:main',     
            'cw1_local_line_intermediate_processor = cw1_team_2.task1.cw1_local_line_intermediate_processor:main',
            'cw1_local_line_processor = cw1_team_2.task1.local_line_processor.py:main',       
        ],
    },
)
