cmake_minimum_required(VERSION 3.8)
project(scanner_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Only generate the JsonIO service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/JsonIO.srv"
)

# Nothing else — remove install() blocks, no Python nodes
ament_package()
