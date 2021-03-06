cmake_minimum_required(VERSION 3.5)
project(vectornav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(libvncxx REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(vectornav_node SHARED src/vectornav_node.cpp)
target_include_directories(
  vectornav_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                        "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(vectornav_node "rclcpp" "rclcpp_components"
                          "libvncxx" "sensor_msgs")
target_compile_definitions(vectornav_node PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(vectornav_node "vn_ros::VectorNavNode")
target_compile_options(
  vectornav_node PRIVATE -Wall -Wextra # reasonable and standard
                         -Wpedantic # warn if non-standard C++ is used
)

install(
  TARGETS vectornav_node
  EXPORT export_vectornav_node
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
