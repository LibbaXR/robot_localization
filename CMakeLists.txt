cmake_minimum_required(VERSION 3.5)
project(robot_localization)

# Add an option to enable/disable building the test
option(BUILD_TEST "Build the test" OFF)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(Eigen3 REQUIRED)

set(library_name rl_lib)

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(
  ${library_name}
  src/ekf.cpp
  src/ukf.cpp
  src/filter_base.cpp
  src/filter_utilities.cpp
  src/robot_localization_estimator.cpp
)

target_link_libraries(
  ${library_name}
  ${EIGEN3_LIBRARIES}
)

if(BUILD_TEST)
  add_executable(filter_record
    filter_record/filter_record.cpp
  )
  target_link_libraries(
    filter_record
    ${library_name}
  )
  
endif()