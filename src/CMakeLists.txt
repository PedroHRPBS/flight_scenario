cmake_minimum_required(VERSION 2.8.3)
project(flight_scenario_manager)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -pthread -g -O0")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

FILE(GLOB SOURCE_FILES *.hpp *.cpp ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

include_directories(/opt/ros/melodic/include)

