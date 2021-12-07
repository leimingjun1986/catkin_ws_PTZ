cmake_minimum_required(VERSION 3.0.2)
project(serial_ptz)

add_compile_options(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  actionlib_msgs
  actionlib
  message_generation
  geometry_msgs
 
)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   SerialPTZ.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
     DEPENDENCIES
     std_msgs
     geometry_msgs
     actionlib_msgs
)


catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES  ${PROJECT_NAME}
    CATKIN_DEPENDS  roscpp serial std_msgs  actionlib_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/serial_ptz.cpp
 )


#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node  src/serial_ptz_node.cpp)

#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   yaml-cpp
)


