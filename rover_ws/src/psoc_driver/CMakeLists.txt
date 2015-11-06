cmake_minimum_required(VERSION 2.8.12)

project(psoc_driver)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  message_generation
)

add_message_files(FILE Drive.msg)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)
find_package(message_generation)

# add package modules path, not needed in dependend packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
include(EnableCXX11)

include_directories(include)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velocity_planner
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_executable(psoc_driver_node
                      src/psoc_node.cpp
                      src/psoc.cpp
#                      include/psoc.h
                      src/conn_interface.cpp
#                      include/conn_interface.h
                      src/conn_serial.cpp
#                      include/conn_serial.h
#                      include/conn_msgbuffer.h
                      )

target_link_libraries(psoc_driver_node ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})
