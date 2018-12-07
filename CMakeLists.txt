cmake_minimum_required(VERSION 2.8.3)
project(bagmetti)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
