from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'benchmark_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itsmevjnk',
    maintainer_email='ngtv0404@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'timer = benchmark_tools.timer:main',
            'wait_until_nav_complete = benchmark_tools.wait_until_nav_complete:main',
            'wait_until_nav_start = benchmark_tools.wait_until_nav_start:main',
            'bumper_node = benchmark_tools.bumper_node:main'
        ],
    },
)
