from setuptools import setup
import os
from glob import glob

package_name = 'prog_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julien',
    maintainer_email='julien.merand@etu.imt-nord-europe.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pub_to_arduino = prog_robot.pub_to_arduino:main",
            "move = prog_robot.move:main",
            "go_home = prog_robot.go_home:main",
        ],
    },
)
