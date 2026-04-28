cmake_minimum_required(VERSION 3.22)
project(alicia_d_descriptions)

find_package(ament_cmake REQUIRED)

# Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install mesh files
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
