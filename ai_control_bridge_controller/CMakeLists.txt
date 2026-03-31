cmake_minimum_required(VERSION 3.16)
project(ai_control_bridge_controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release build
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Enable optimizations
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
endif()

# Warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ai_control_bridge_core REQUIRED)

# Header-only library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
    ai_control_bridge_core::ai_control_bridge_core
)

# Implementation library with controller plugin
add_library(${PROJECT_NAME}_plugin SHARED
    src/controller.cpp
    src/trajectory.cpp
    src/safety.cpp
    src/buffer.cpp
)

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME}
    ai_control_bridge_core::ai_control_bridge_core
)

ament_target_dependencies(${PROJECT_NAME}_plugin
    rclcpp
    rclcpp_lifecycle
    controller_interface
    hardware_interface
    pluginlib
    sensor_msgs
    std_msgs
    geometry_msgs
)

# Register the controller plugin
pluginlib_export_plugin_description_file(
    controller_interface 
    ai_control_bridge_controller_plugin.xml
)

# Install targets
install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
    FILES ai_control_bridge_controller_plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ai_control_bridge_core
    rclcpp
    rclcpp_lifecycle
    controller_interface
    hardware_interface
    pluginlib
)

ament_package()
