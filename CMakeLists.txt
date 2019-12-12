cmake_minimum_required(VERSION 2.8.3)
project(flight_scenario)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)


FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
