import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'part2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='app14',
    maintainer_email='dlehddn14785@gmail.com',
    description='Part 2 - frames, transforms, FK and IK',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frame_converter = part2_pkg.frame_converter:main',
            'fk_check = part2_pkg.fk_check:main',
            'ik_solve = part2_pkg.ik_solve:main',
            'mobile_frames = part2_pkg.mobile_frames:main',
            'odom_drift = part2_pkg.odom_drift:main',
        ],
    },
)
