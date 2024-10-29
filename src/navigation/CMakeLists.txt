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
find_package(PkgConfig REQUIRED)
pkg_check_modules(CAIRO REQUIRED cairo)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)


include_directories(${GEOGRAPHICLIB_INCLUDE_DIRS})


add_executable(navigation src/navigation.cpp src/aqua_utils.cpp)
ament_target_dependencies(navigation rclcpp std_msgs sensor_msgs geometry_msgs tf2)


add_executable(graph_node src/graph_node.cpp) 
ament_target_dependencies(graph_node rclcpp std_msgs sensor_msgs geometry_msgs tf2)
target_include_directories(graph_node PRIVATE ${CAIRO_INCLUDE_DIRS})
target_link_libraries(graph_node ${CAIRO_LIBRARIES} ${GEOGRAPHICLIB_LIBRARIES})


add_executable(graph_node src/graph_node.cpp) 
ament_target_dependencies(graph_node rclcpp std_msgs sensor_msgs geometry_msgs tf2)


target_include_directories(navigation PRIVATE src)

install(TARGETS
  navigation
  graph_node
  mtr_control
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()

