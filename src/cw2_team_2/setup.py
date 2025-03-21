from setuptools import find_packages, setup

package_name = 'cw2_team_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'foot_contact_estimator = cw2_team_2.task1.foot_contact_estimator:main',
            'cw2_goalpose_follower = cw2_team_2.task2.cw2_goalpose_follower:main',
        ],
    },
)
