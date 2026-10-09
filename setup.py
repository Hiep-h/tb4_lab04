import os
from glob import glob
from setuptools import setup

package_name = 'tb4_lab04'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vu Van Hiep',
    maintainer_email='hiepga05102005@gmail.com',
    description='Lab 04: TurtleBot 4 simulation, SLAM, localization and Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_survey = tb4_lab04.auto_survey:main',
            'control_panel = tb4_lab04.control_panel:main',
            'auto_explore = tb4_lab04.auto_explore:main',
        ],
    },
)
