cmake_minimum_required(VERSION 2.8.3)
project(task_goto_multiple)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV
find_package( OpenCV REQUIRED )
include_directories(${OPENCV_INCLUDE_DIRS})


#Boost
FIND_PACKAGE( Boost 1.40 COMPONENTS system filesystem REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

include_directories(libs/include)

SET(SOURCES1
    libs/include/task_goto_seq.hpp
    libs/include/ROSNode_GoTo.hpp
    libs/include/action_t.hpp
    libs/include/skill_t.hpp
    libs/include/task_t.hpp
    libs/include/action_setvw.hpp
    libs/src/action_setvw.cpp
    libs/include/skill_goto.hpp
    libs/src/skill_goto.cpp
    libs/src/task_goto_seq.cpp
    libs/src/ROSNode_GoTo.cpp
)




#QT
set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5Network
)

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::Network
  Qt5::Core
  yaml-cpp
)


foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

#find_package(Qt5Widgets REQUIRED)
#find_package(Qt5Network REQUIRED)


#ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp tf roslib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################

add_message_files(
    DIRECTORY
       msg
    FILES
       vel_cmd.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    #INCLUDE_DIRS libs/include 
    CATKIN_DEPENDS roscpp std_srv message_runtime
    DEPENDS ${EXTERNAL_DEPENDS}
)

###
### QT stuff (required if you want a GUI)
###
set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files

###########
## Build ##
###########

## Declare a C++ executable
add_executable(task_goto_multiple_node task_goto_multiple.cpp ${SOURCES1})

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(task_goto_multiple_node PROPERTIES OUTPUT_NAME task_goto_multiple_node PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(task_goto_multiple_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(task_goto_multiple_node ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${EXTERNAL_DEPENDS_LIB})



#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS task_goto_multiple_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/ libs/include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
