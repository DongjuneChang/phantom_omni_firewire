cmake_minimum_required(VERSION 3.8)
project(omni_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
