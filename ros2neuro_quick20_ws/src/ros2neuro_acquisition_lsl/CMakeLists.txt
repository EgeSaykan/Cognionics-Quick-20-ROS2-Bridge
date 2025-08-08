cmake_minimum_required(VERSION 3.8)
project(ros2neuro_acquisition_lsl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2neuro_data REQUIRED)
find_package(ros2neuro_msgs REQUIRED)
find_package(ros2neuro_acquisition REQUIRED)

find_package(GTest REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

find_library(LSL_LIBRARIES lsl REQUIRED)
find_path(LSL_INCLUDE_DIRS lsl_cpp.h REQUIRED)

if(NOT LSL_LIBRARIES)
    message(WARNING "Library LSL not found. The plugin will not be compiled.")
else()
    message(STATUS "Library LSL found. The plugin will be installed.")

    include_directories(
        include
        include/${PROJECT_NAME}
        ${LSL_INCLUDE_DIRS}
    )

    add_library(${PROJECT_NAME} SHARED
        src/LSLDevice.cpp
    )

    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        pluginlib
        ros2neuro_data
        ros2neuro_msgs
        ros2neuro_acquisition
    )

    target_link_libraries(${PROJECT_NAME} ${LSL_LIBRARIES})

    install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

    install(FILES plugin_lsl.xml
        DESTINATION share/${PROJECT_NAME})

    # Test
    ament_add_gtest(test_lsl_device test/TestLslDevice.cpp)
    target_link_libraries(test_lsl_device ${PROJECT_NAME} ${LSL_LIBRARIES})
    include_directories(${GTEST_INCLUDE_DIRS})
endif()

ament_package()
