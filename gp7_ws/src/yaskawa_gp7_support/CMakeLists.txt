cmake_minimum_required(VERSION 3.8)
project(yaskawa_gp7_support)

find_package(ament_cmake REQUIRED)

install(
	DIRECTORY config meshes srdf urdf
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
