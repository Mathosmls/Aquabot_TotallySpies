cmake_minimum_required(VERSION 3.5)
project(array2stamp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

include_directories(${GEOGRAPHICLIB_INCLUDE_DIRS})

include_directories(include)

# Définir l'exécutable
add_executable(array2stamp_node src/array2stamp.cpp)

# Lier les dépendances
ament_target_dependencies(array2stamp_node rclcpp geometry_msgs sensor_msgs)

target_link_libraries(array2stamp_node ${GEOGRAPHICLIB_LIBRARIES})

# Installer l'exécutable
install(TARGETS
  array2stamp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

