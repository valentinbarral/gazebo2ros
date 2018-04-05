cmake_minimum_required(VERSION 2.8.3)
project(gtec_gazebo2ros)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")

add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  gtec_msgs
)

## System dependencies are found with CMake's conventions

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})


include_directories(src/publishers)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
)

set(SOURCE_FILES_GAZEBO_TO_ROS
    src/publishers/Gazebo2ROS.cpp
     src/publishers/opticalFlow.pb.cc)

set(SOURCE_FILES_GAZEBO_GUIDE
    src/publishers/GazeboGuide.cpp)

set(SOURCE_FILES_GAZEBO_GUIDE_RTLS
    src/publishers/GazeboGuideRTLS.cpp)

set(SOURCE_FILES_GAZEBO_AUTO_GUIDE
    src/publishers/GazeboAutoGuide.cpp)

set(SOURCE_FILES_GAZEBO_RESTART_SERVICE
    src/publishers/GazeboRestartService.cpp)


## Declare a cpp executable

add_executable(gazebo2ros ${SOURCE_FILES_GAZEBO_TO_ROS})
add_dependencies(gazebo2ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo2ros ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(gazebo_guide ${SOURCE_FILES_GAZEBO_GUIDE})
add_dependencies(gazebo_guide ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_guide ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(gazebo_guide_rtls ${SOURCE_FILES_GAZEBO_GUIDE_RTLS})
add_dependencies(gazebo_guide_rtls ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_guide_rtls ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(gazebo_auto_guide ${SOURCE_FILES_GAZEBO_AUTO_GUIDE})
add_dependencies(gazebo_auto_guide ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_auto_guide ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(gazebo_restart_service ${SOURCE_FILES_GAZEBO_RESTART_SERVICE})
add_dependencies(gazebo_restart_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_restart_service ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})