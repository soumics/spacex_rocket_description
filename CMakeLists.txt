cmake_minimum_required(VERSION 3.8)
project(spacex_rocket_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf meshes vehicles rviz materials
  DESTINATION share/${PROJECT_NAME}
)

# Lets Gazebo resolve package://spacex_rocket_description/... (as model://...) URIs
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
