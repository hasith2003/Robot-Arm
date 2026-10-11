from setuptools import find_packages, setup

package_name = 'gp7_task'
setup(name=package_name, version='0.1.0', packages=find_packages(), install_requires=['setuptools', 'numpy', 'scipy'], zip_safe=True, maintainer='EN3563 robotics team', maintainer_email='team@example.com', description='GP7 task planning primitives.', license='Apache-2.0', data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])], tests_require=['pytest'], entry_points={'console_scripts': ['joint_motion_demo = gp7_task.joint_motion_demo:main', 'gripper_demo = gp7_task.gripper_demo:main', 'pick_place = gp7_task.pick_place:main']})
