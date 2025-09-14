from setuptools import setup
import os
from glob import glob

package_name = 'tb3_fusion_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
	('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skaigroove',
    maintainer_email='skaigroove@gmail.com',
    description='TB3 RGBD cam + TF + sync + bringup launcher',
    license='MIT',
)
