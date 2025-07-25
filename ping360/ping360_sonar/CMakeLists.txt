cmake_minimum_required(VERSION 3.5)
project(ping360_sonar)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ping360_sonar_msgs REQUIRED)
find_package(image_transport REQUIRED)

# build BlueRobotics' ping-cpp  standalone    
add_subdirectory(src)

# main code is a node class + standalone executable
add_executable(ping360_node src/ping360_node.cpp src/ping360.cpp src/sonar_interface.cpp src/sector.cpp)
# avoid this warning from BlueRobotics' ping-cpp
target_compile_options(ping360_node PRIVATE "-Wno-unused-function")
if(REAL_SONAR)
    target_compile_definitions(ping360_node PUBLIC REAL_SONAR)
endif()
target_include_directories(ping360_node PRIVATE include src/ping-cpp/src)
target_link_libraries(ping360_node ping-cpp)
ament_target_dependencies(ping360_node rclcpp sensor_msgs ping360_sonar_msgs image_transport)
  
# Install
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS src/ping360.py DESTINATION lib/${PROJECT_NAME})
#install(DIRECTORY python/ping360_sonar DESTINATION lib/${PROJECT_NAME})

install(TARGETS ping360_node ping-cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
