cmake_minimum_required(VERSION 2.8.3)
project(robot_drivers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  turtlesim
  roscpp
  rospy
  tf
)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  libraries/EPOS_Linux_Library/include
  includes
)

## Declare a C++ library
add_library(${PROJECT_NAME}_LIB
  src/drive_train_manager.cpp
  src/epos_drive_manager.cpp
  src/pid.cpp
  # src/tests/drive_train_manager_test.cpp
  # src/tests/epos_drive_manager_test.cpp
)

link_directories(
  libraries/boost
)

target_link_libraries(${PROJECT_NAME}_LIB
  ${catkin_LIBRARIES}
  EposCmd
)

add_executable(drive_train_ros_handler_node src/drive_train_ros_handler.cpp)
target_link_libraries(drive_train_ros_handler_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_LIB
)
add_dependencies(drive_train_ros_handler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(test_epos_node src/tests/test_epos.cpp)
# target_link_libraries(test_epos_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}_LIB
# )
# add_dependencies(test_epos_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(test_drive_train_node src/tests/test_drive_train.cpp)
# target_link_libraries(test_drive_train_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}_LIB
# )
# add_dependencies(test_drive_train_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(test_pid_node src/tests/test_pid.cpp)
# target_link_libraries(test_pid_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}_LIB
# )
# add_dependencies(test_pid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(test_drive_train_tune_pid_node src/tests/test_drive_train_tune_pid.cpp)
# target_link_libraries(test_drive_train_tune_pid_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}_LIB
# )
# add_dependencies(test_drive_train_tune_pid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(practice_epos_control_node src/practice_epos_control.cpp)
target_link_libraries(practice_epos_control_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_LIB
)
add_dependencies(practice_epos_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(drive_train_manager_node src/drive_train_manager.cpp)
# target_link_libraries(drive_train_manager_node
#   ${catkin_LIBRARIES}
#   EposCmd
# )
# add_dependencies(drive_train_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(epos_drive_manager_node src/epos_drive_manager.cpp)
# target_link_libraries(epos_drive_manager_node
#   ${catkin_LIBRARIES}
#   EposCmd
# )
# add_dependencies(epos_drive_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(drive_train_manager_test_node src/tests/drive_train_manager_test.cpp)
# target_link_libraries(${PROJECT_NAME}_LIB
#   ${catkin_LIBRARIES}
#   EposCmd
# )
# add_dependencies(drive_train_manager_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(${PROJECT_NAME}_node src/tests/epos_drive_manager_test.cpp)
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   EposCmd
# )
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# set (dependencies apple bananas carrots)
# foreach(dependency $(dependencies))
# 	message("$(dependencies) has item $(dependency)")
# endforeach()