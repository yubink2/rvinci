cmake_minimum_required(VERSION 3.0.2)
project(interactive_marker_tutorials)

find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp visualization_msgs tf)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_controls src/basic_controls.cpp)
target_link_libraries(basic_controls
   ${catkin_LIBRARIES}
)

add_executable(button_mode src/button_mode.cpp)
target_link_libraries(selection
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rvinci
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)