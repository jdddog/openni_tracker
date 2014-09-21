cmake_minimum_required(VERSION 2.8.3)

project(openni_tracker)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
					roscpp
					geometry_msgs
					roslib
					tf
					std_msgs)
# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   User.msg
   UserList.msg
 )

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   MotorRange.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

# Find Nite
find_path(Nite_INCLUDEDIR
	  NAMES XnVNite.h
	  HINTS /usr/include/nite /usr/local/include/nite)
find_library(Nite_LIBRARY
	     NAMES XnVNite_1_3_1
	     HINTS /usr/lib /usr/local/lib
	     PATH_SUFFIXES lib) 

catkin_package()

include_directories(${catkin_INCLUDEDIR}
		    ${OpenNI_INCLUDEDIR}
		    ${Nite_INCLUDEDIR}
		    ${orocos_kdl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(openni_tracker src/openni_tracker.cpp)

add_dependencies(openni_tracker geometry_msgs_gencpp)

target_link_libraries(openni_tracker ${catkin_LIBRARIES}
				     ${OpenNI_LIBRARIES}
				     ${Nite_LIBRARY}
				     ${orocos_kdl_LIBRARIES})

install(TARGETS openni_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
