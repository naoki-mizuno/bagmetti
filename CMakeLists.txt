cmake_minimum_required(VERSION 2.8.3)
project(bagmetti)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/filter.test)
  add_rostest(test/rename.test)
endif()
