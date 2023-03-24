cmake_minimum_required(VERSION 3.5.1)
project(torque_controller VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)

include_directories("${PROJECT_SOURCE_DIR}/include/")

set(SOURCES
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  ${PROJECT_SOURCE_DIR}/src/controller_data.cpp
  ${PROJECT_SOURCE_DIR}/src/pid.cpp
  ${PROJECT_SOURCE_DIR}/src/filter.cpp
  ${PROJECT_SOURCE_DIR}/src/sweeper_chassis_controller.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
