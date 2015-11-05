cmake_minimum_required(VERSION 2.8.3)
project(nostop_agent)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs message_generation base_math nostop_learning tf)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   GuardSensorCtrl.msg
   GuardState.msg
   ThiefState.msg
   Shape.msg
   Id_robot.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   PlayerIDData.srv
   AreaData.srv
   GuardBenefitData.srv
   GuardNeighboursData.srv
   PlayerConfigurationData.srv
   PlayerNotifyStatus.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   nav_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  DEPENDS base_math nostop_learning
)

###########
## Build ##
###########

include_directories(include include/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
include_directories(src/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_INSTALL_PREFIX}/include) # this line is to find lemon include files.

set(guard_SOURCES
        src/AgentProcess.cpp
        src/AreaCreator.cpp
        src/Configuration.cpp
        src/Conversions.cpp
        src/Guard.cpp
        src/GuardNeighbours.cpp
        src/GuardProcess.cpp
        src/GuardStatePublisher.cpp
        src/iAgent.cpp
        src/iGuard.cpp
        src/iLocalizer.cpp
        src/LearningInitializer.cpp
        src/LearningProcess.cpp
        src/MonitorReceiver.cpp
        src/MotorControl.cpp
        src/StatePublisher.cpp
        src/StateUpdater.cpp
        src/WorldMap.cpp
)

set(thief_SOURCES
	src/AgentProcess.cpp
        src/AreaCreator.cpp
        src/Configuration.cpp
        src/Conversions.cpp
        src/iAgent.cpp
        src/iLocalizer.cpp
        src/iThief.cpp
        src/LearningInitializer.cpp
        src/MotorControl.cpp
        src/StatePublisher.cpp
        src/StateUpdater.cpp
        src/WorldMap.cpp
        src/Thief.cpp
        src/ThiefProcess.cpp
        src/ThiefStatePublisher.cpp
)

set(agent_SOURCES
	src/AgentProcess.cpp
        src/AreaCreator.cpp
        src/Configuration.cpp
        src/Conversions.cpp
        src/GuardNeighbours.cpp
        src/GuardProcess.cpp
        src/GuardStatePublisher.cpp
        src/iAgent.cpp
        src/iGuard.cpp
        src/iLocalizer.cpp
        src/iThief.cpp
        src/LearningInitializer.cpp
        src/LearningProcess.cpp
        src/MonitorReceiver.cpp
        src/MotorControl.cpp
        src/StatePublisher.cpp
        src/StateUpdater.cpp
        src/ThiefProcess.cpp
        src/ThiefStatePublisher.cpp
        src/WorldMap.cpp
)

add_executable(Guard ${guard_SOURCES})
target_link_libraries(Guard ${catkin_LIBRARIES} nostop_learning base_math lemon)
add_dependencies(Guard ${PROJECT_NAME}_gencpp)

add_executable(Thief ${thief_SOURCES})
target_link_libraries(Thief ${catkin_LIBRARIES} nostop_learning base_math lemon)
add_dependencies(Thief ${PROJECT_NAME}_gencpp)

add_library(${PROJECT_NAME} SHARED ${agent_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} nostop_learning base_math lemon)
add_dependencies(${PROJECT_NAME} nostop_agent_gencpp nostop_learning base_math lemon)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS Guard Thief ${PROJECT_NAME}
 EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT lib
   RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT bin
   LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}" COMPONENT shlib
 )