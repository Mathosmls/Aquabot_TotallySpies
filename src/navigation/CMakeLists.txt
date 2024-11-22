cmake_minimum_required(VERSION 3.5)
project(navigation)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
set(nav2_costmap_2d_DIR "/opt/ros/humble/share/nav2_costmap_2d/cmake")
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_bringup REQUIRED)

pkg_check_modules(CAIRO REQUIRED cairo)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

include_directories(${GEOGRAPHICLIB_INCLUDE_DIRS})


# Définir l'exécutable graph_node (une seule fois)
add_executable(graph_node src/graph_node.cpp)
ament_target_dependencies(graph_node rclcpp std_msgs sensor_msgs geometry_msgs tf2)
target_include_directories(graph_node PRIVATE ${CAIRO_INCLUDE_DIRS})
target_link_libraries(graph_node ${CAIRO_LIBRARIES} ${GEOGRAPHICLIB_LIBRARIES})

install(TARGETS
  graph_node # Assurez-vous que cette cible est définie
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

