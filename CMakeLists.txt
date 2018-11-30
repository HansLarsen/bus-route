cmake_minimum_required(VERSION 2.8.3)
project(busroute)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  interactive_markers 
  visualization_msgs 
  tf 
  geometry_msgs 
  move_base_msgs 
  actionlib
  )

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(busroute_node src/busroute.cpp)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES busroute
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


## Specify libraries to link a library or executable target against
target_link_libraries(busroute_node
  ${catkin_LIBRARIES}
)
