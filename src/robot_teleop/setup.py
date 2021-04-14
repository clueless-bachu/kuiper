import os
from glob import glob
from setuptools import setup

package_name = 'robot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='vasista1997@gmail.com',
    description='Controls the robot via the keyboard',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = robot_teleop.teleop_keyboard:main',
            'teleop_internet = robot_teleop.teleop_internet:main'
        ],
    },
)
