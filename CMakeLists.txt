cmake_minimum_required(VERSION 2.8)
project(lidar_object_detection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")

# Try to find Point Cloud Library
find_package(PCL 1.2 REQUIRED)
if (PCL_FOUND)
    message("Find PCL")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
else ()
    message(FATAL_ERROR "Could not find PCL")
endif()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_conversions
        message_generation
        )
if (catkin_FOUND)
    message("Find catkin")
    add_message_files(FILES bbox.msg)
    generate_messages(DEPENDENCIES std_msgs sensor_msgs)
    catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime)
    include_directories(${catkin_INCLUDE_DIRS})
else ()
    message(FATAL_ERROR "Could not find catkin")
endif()

file(GLOB SOURCE_FILES src/*.cpp src/render/*.cpp)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries (${CMAKE_PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}_generate_messages_cpp)