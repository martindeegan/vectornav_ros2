cmake_minimum_required(VERSION 3.5)
project(libvncxx CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

add_library(
  vncxx SHARED
  src/attitude.cpp
  src/compositedata.cpp
  src/conversions.cpp
  src/criticalsection.cpp
  src/dllvalidator.cpp
  src/error_detection.cpp
  src/event.cpp
  src/ezasyncdata.cpp
  src/memoryport.cpp
  src/packet.cpp
  src/packetfinder.cpp
  src/port.cpp
  src/position.cpp
  src/searcher.cpp
  src/sensors.cpp
  src/serialport.cpp
  src/thread.cpp
  src/types.cpp
  src/util.cpp
  src/utilities.cpp
  src/vntime.cpp)

target_include_directories(
  vncxx SYSTEM PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                      "$<INSTALL_INTERFACE:include>")

ament_export_include_directories(include)
ament_export_libraries(vncxx)
ament_export_interfaces(export_libvncxx HAS_LIBRARY_TARGET)

install(
  TARGETS vncxx
  EXPORT export_libvncxx
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

ament_package()
