from glob import glob
from setuptools import find_packages, setup

package_name = 'gp7_simulation'
setup(name=package_name, version='0.1.0', packages=find_packages(exclude=['test']), data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']), ('share/' + package_name + '/launch', glob('launch/*.launch.py')), ('share/' + package_name + '/worlds', glob('worlds/*.sdf')), ('share/' + package_name + '/config', glob('config/*.yaml'))], install_requires=['setuptools'], zip_safe=True, maintainer='EN3563 robotics team', maintainer_email='team@example.com', description='Gazebo simulation assets for the GP7 project.', license='Apache-2.0')
