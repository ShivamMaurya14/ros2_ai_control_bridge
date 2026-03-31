cmake_minimum_required(VERSION 3.16)
project(ai_inference_processor CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_control_bridge_core REQUIRED)

# Header library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
    ai_control_bridge_core::ai_control_bridge_core
)

# Node executable
add_executable(${PROJECT_NAME}_node
    src/inference_processor.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ai_control_bridge_core::ai_control_bridge_core
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    std_msgs
)

# Install
install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
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
    sensor_msgs
    std_msgs
)

ament_package()
