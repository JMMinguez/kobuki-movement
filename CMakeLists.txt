cmake_minimum_required(VERSION 3.8)
project(go_forward)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
    kobuki_ros_interfaces
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/go_forward_functions/functions.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(forward_node src/program.cpp)
ament_target_dependencies(forward_node ${dependencies})
target_link_libraries(forward_node ${PROJECT_NAME})

install(TARGETS
  forward_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
