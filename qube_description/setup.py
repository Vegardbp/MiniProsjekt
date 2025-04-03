from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qube_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #installerer launch.py filene fra launch mappen
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        #instalerer alle urdf/xacro filene
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro*')),
        #instalerer alle rviz konfigurasjonsfiler
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='103881675+Vegardbp@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
