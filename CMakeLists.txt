cmake_minimum_required(VERSION 3.5)
project(livox_free_space)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED COMPONENTS )
find_package(rclcpp REQUIRED)
       # rospy
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED PATHs /opt/ros/humble)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(patchworkpp REQUIRED)

       # message_generation

include_directories(
        include
        include ${PCL_INCLUDE_DIRS}
        ${patchworkpp_INCLUDE_DIRS}
)

link_directories(
        ${PCL_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_node src/FreeSpace_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp patchworkpp PCL pcl_conversions pcl_ros std_msgs visualization_msgs nav_msgs sensor_msgs geometry_msgs tf2_ros)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${PCL_LIBRARIES}
        /usr/local/lib/libground_seg_cores.a
        dl -lboost_system
)

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()

