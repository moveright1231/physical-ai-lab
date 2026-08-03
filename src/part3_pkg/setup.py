import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'part3_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='app14',
    maintainer_email='dlehddn14785@gmail.com',
    description='Part 3 - 액추에이터와 제어: ros2_control 자립 실습 (mock hardware)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_pose = part3_pkg.arm_pose:main',
            'drive_test = part3_pkg.drive_test:main',
            'mm_demo = part3_pkg.mm_demo:main',
        ],
    },
)
