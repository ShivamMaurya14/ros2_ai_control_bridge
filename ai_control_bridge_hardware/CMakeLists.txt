cmake_minimum_required(VERSION 3.16)
project(ai_control_bridge_hardware CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ai_control_bridge_core REQUIRED)

# Header library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
    ai_control_bridge_core::ai_control_bridge_core
    rclcpp::rclcpp
    hardware_interface::hardware_interface
    controller_interface::controller_interface
    pluginlib::pluginlib
)

# Implementation library for BCR ARM hardware interface
add_library(${PROJECT_NAME}_impl SHARED
    src/bcr_arm_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME}_impl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}_impl
    ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_impl
    rclcpp
    hardware_interface
)

# Install
install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_impl
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
    DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ai_control_bridge_core
    rclcpp
    hardware_interface
)

ament_package()
