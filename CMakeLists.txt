cmake_minimum_required(VERSION 3.5)
project(anyfeeder_connector)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon in CLion
# Ref: https://github.com/clydemcqueen/flock2/blob/master/CMakeLists.txt
if($ENV{CLION_IDE})
  message("Running inside CLion")
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/eloquent/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/eloquent/lib/libfastrtps.so")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(anyfeeder_interfaces REQUIRED)

set(dependencies
  rclcpp
  std_srvs
  anyfeeder_interfaces
)

include_directories(include)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED src/anyfeeder.cpp src/serial_connection.cpp src/mock_connection.cpp src/ethernet_connection.cpp)
ament_target_dependencies(${library_name} ${dependencies})

add_executable(anyfeeder_node src/anyfeeder_node.cpp)
target_link_libraries(anyfeeder_node ${library_name})
ament_target_dependencies(anyfeeder_node ${dependencies})


install(TARGETS anyfeeder_node ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}-test
    test/test_anyfeeder.cpp
    test/test_actions.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

###########
ament_package()
