cmake_minimum_required(VERSION 3.5)
project(array2stamp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Définir l'exécutable
add_executable(array2stamp_node src/array2stamp_node.cpp)

# Lier les dépendances
ament_target_dependencies(array2stamp_node rclcpp geometry_msgs)

# Installer l'exécutable
install(TARGETS
  array2stamp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

