cmake_minimum_required(VERSION 3.8)
project(gazebo_usar_gui)

###################
## Preprocessing ##
###################

## QT automoc is ON
set(CMAKE_AUTOMOC ON)

## Declare compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    gazebo_ros
    roscpp
    gazebo_usar_core
)
## Find system dependencies
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
find_package(Qt5 5.9 REQUIRED
  COMPONENTS
    Widgets
    Core
)
## Find Qt libraries manually
find_library(Qt5Core_LIBRARY
  NAMES
    Qt5Core libQt5Core
  PATHS "/usr/lib/x86_64-linux-gnu"
  NO_DEFAULT_PATH
)
find_library(Qt5Widgets_LIBRARY
  NAMES
    Qt5Widgets libQt5Widgets
  PATHS "/usr/lib/x86_64-linux-gnu"
  NO_DEFAULT_PATH
)

## Declared as a catkin package
catkin_package()

## Prepare sources
file(GLOB SOURCES src/*.cc)
file(GLOB QT_MOC include/gazebo_usar_gui.hh)

## Wrap sources using QT Moc
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations (include) should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${GAZEBO_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Specify the directory in which the linker will look for Gazebo libraries
link_directories(${GAZEBO_LIBRARY_DIRS})

## Declare a C++ library
add_library(gazebo_usar_gui SHARED ${QT_MOC_HPP} ${SOURCES})

## Add cmake target dependencies of the library
add_dependencies(gazebo_usar_gui ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library target against
target_link_libraries(gazebo_usar_gui
    ${GAZEBO_LIBRARIES}
    # ${Qt5Core_LIBRARY}
    # ${Qt5Widgets_LIBRARY}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
