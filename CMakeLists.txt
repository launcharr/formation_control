cmake_minimum_required(VERSION 2.8.3)
project(formation_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  navcon_msgs
  roscpp
  rospy
  message_generation
)

add_message_files(
  FILES
  Formation.msg
)

add_service_files(
  FILES
  FormChange.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

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
#  LIBRARIES formation_control
	CATKIN_DEPENDS auv_msgs navcon_msgs roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(FormationControl src/FormationControl.cpp)
add_executable(CurrentAdaptiveFormation src/CurrentAdaptiveFormation.cpp)

target_link_libraries(FormationControl ${catkin_LIBRARIES})
target_link_libraries(CurrentAdaptiveFormation ${catkin_LIBRARIES})

if (DEFINED catkin_EXPORTED_TARGETS)
	add_dependencies(FormationControl ${catkin_EXPORTED_TARGETS})
    add_dependencies(CurrentAdaptiveFormation ${catkin_EXPORTED_TARGETS})
endif()



