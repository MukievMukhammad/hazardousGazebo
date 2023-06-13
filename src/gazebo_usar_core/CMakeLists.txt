cmake_minimum_required(VERSION 2.8.3)
project(gazebo_usar_core)

###################
## Preprocessing ##
###################

## Specify a name of the library
set(LIBRARY_NAME "gazebo_usar_core")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-fopenmp)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    gazebo_ros
    tf
    std_msgs
    nav_msgs
    message_generation
    pcl_conversions
    pcl_ros
    sensor_msgs
    move_base_msgs
    actionlib
    dynamic_reconfigure
)
## Find system dependencies
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    system filesystem thread
)
find_package(Qt5 5.9 REQUIRED
  COMPONENTS
    Core
)
## Find Qt libraries manually
find_library(Qt5Core_LIBRARY
  NAMES
    Qt5Core libQt5Core
  PATHS "/usr/lib/x86_64-linux-gnu"
  NO_DEFAULT_PATH
)
#find_package(SDL2 REQUIRED)
#INCLUDE(FindPkgConfig)
#PKG_SEARCH_MODULE(SDL2MIXER REQUIRED SDL2_mixer>=2.0.0)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    Command.msg
    RadiationSubstance.msg
    RadiationSubstances.msg
    ChemicalSubstance.msg
    ChemicalSubstances.msg
    ChemicalConcentration.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    gazebo_usar_core
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/Contamination.cfg
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging
  ${Qt5Core_INCLUDE_DIRS}
  #${SDL2_INCLUDE_DIRS}
  #${SDL2MIXER_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

## Declare a C++ library
add_library(${LIBRARY_NAME} SHARED
   src/hazardouszones.cpp
   src/gazebo_usar_core.cpp
   #src/gazebo_audio.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${LIBRARY_NAME} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(${LIBRARY_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  # ${Qt5Core_LIBRARY}
  #${SDL2_LIBRARIES}
  #${SDL2MIXER_LIBRARIES}
)
