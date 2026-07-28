import os
from glob import glob
from setuptools import setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='app14',
    maintainer_email='dlehddn14785@gmail.com',
    description='Physical AI Lab Part 1 practice package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_pkg.my_node:main',
            'talker = my_pkg.talker:main',
            'listener = my_pkg.listener:main',
            'add_server = my_pkg.add_server:main',
            'param_node = my_pkg.param_node:main',
            'dummy_robot = my_pkg.dummy_robot:main',
            'joint_reader = my_pkg.joint_reader:main',
            'joint_commander = my_pkg.joint_commander:main',
        ],
    },
)
