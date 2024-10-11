from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sairamu02',
    maintainer_email='sairam@u.northwestern.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "waypoint = turtle_control.waypoint:main",
        ],
    },
)
