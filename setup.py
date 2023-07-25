from setuptools import setup
import os
from glob import glob

package_name = 'calibration_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BELIER Titouan',
    maintainer_email='titouan.belier@ensta-bretagne.org',
    description='This ROS2 package allows user to calibrate a LiDAR-camera system. The user only has to modify the name of the topics \
                of its components and the path of the folder where it wants to save images and pointclouds.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration             = calibration_pkg.calibration:main',
            'get_ext_param           = calibration_pkg.get_ext_param:main',
            'open_modify_pointcloud  = calibration_pkg.open_modify_pointcloud:main',
            'open_modify_image       = calibration_pkg.open_modify_image:main',
            'save_pointcloud         = calibration_pkg.save_pointcloud:main',
            'save_image              = calibration_pkg.save_image:main',
        ],
    },
)