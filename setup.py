from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vicon_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Haoru Xue',
    maintainer_email='haorux@andrew.cmu.edu',
    description='interface with vicon',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vicon_stream_node = vicon_interface.vicon_stream_node:main'
        ],
    },
)
