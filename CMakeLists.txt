cmake_minimum_required(VERSION 2.8.3)
project(ann1vn)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(DIRECTORY nav1
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS nodes/ann_recognizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS nodes/voice_cmd_vel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
