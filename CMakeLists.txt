cmake_minimum_required(VERSION 3.5)
project(bagmetti)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
    scripts/filter.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
