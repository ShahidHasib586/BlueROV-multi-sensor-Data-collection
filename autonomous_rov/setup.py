from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonomous_rov'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),        
        (os.path.join('share', package_name, 'config'), glob('config/*.config.yaml')),
        (os.path.join('share', package_name, 'lib'), glob('*.py')),                                                
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shahid Ahamed Hasib',
    maintainer_email='shahidhasib586@gmail.com',
    description='Autonomous ROV Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'listener = autonomous_rov.listener:main',
        'video = autonomous_rov.video:main',
        'pinger_node = autonomous_rov.pinger_node:main',
        'synced_node = autonomous_rov.synced_node:main',
        'image_processing_tracker = autonomous_rov.image_processing_tracker:main'
        ],       
        
    },
)
