from setuptools import find_packages, setup

package_name = 'gp7_kinematics'
setup(name=package_name, version='0.1.0', packages=find_packages(), install_requires=['setuptools', 'numpy'], zip_safe=True, maintainer='EN3563 robotics team', maintainer_email='team@example.com', description='GP7 kinematics library.', license='Apache-2.0', data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])])
