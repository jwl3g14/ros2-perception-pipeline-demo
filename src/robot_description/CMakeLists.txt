cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  meshes
  worlds
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
