cmake_minimum_required(VERSION 2.8.3)
project(rs_bs)
find_package(catkin REQUIRED robosherlock world_control_msgs unreal_interface)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs_bs)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/rs_bs/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)
################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)
catkin_package(
   INCLUDE_DIRS include
   CFG_EXTRAS ${PROJECT_NAME}_config.cmake
   )
################################################################################
## Package dependencies                                                       ##
################################################################################
#Add required catkin packages here
#find_package(catkin REQUIRED COMPONENTS xxx)
#Add required system packages here
#find_package(xxx)
################################################################################
## Build                                                                      ##
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
        )
################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################
## generate classes from the typesystem xml files
generate_type_system(robosherlock rs_bs)
#find all relevant files
find_additional_files()
################################################################################
## Sub Projects                                                               ##
################################################################################
#If you want to divide your projects into subprojects include the subdirectories
#each containing a CMakeLists.txt here
#add_subdirectory(src/xxx)

add_library(rs_BeliefStateCommunication SHARED
        src/BeliefStateCommunication.cpp
        )

add_library(rs_BeliefStateRepresentation SHARED
        src/BeliefStateRepresentation.cpp
        )

target_link_libraries(rs_BeliefStateCommunication ${catkin_LIBRARIES}) # this once caused a super long linking process in clion?
target_link_libraries(rs_BeliefStateRepresentation)

rs_add_library(rs_BeliefStateSensorVisualizer src/BeliefStateSensorVisualizer.cpp)
target_link_libraries(rs_BeliefStateSensorVisualizer ${catkin_LIBRARIES})
rs_add_library(rs_BeliefStateSegmentationAnnotator src/BeliefStateSegmentationAnnotator.cpp)
target_link_libraries(rs_BeliefStateSegmentationAnnotator ${catkin_LIBRARIES} rs_BeliefStateRepresentation)
rs_add_library(rs_BeliefStatePoseRefinement src/BeliefStatePoseRefinement.cpp)
target_link_libraries(rs_BeliefStatePoseRefinement ${catkin_LIBRARIES} rs_CloudSimilarity rs_BeliefStateCommunication)
rs_add_library(rs_BeliefStateCameraUpdater src/BeliefStateCameraUpdater.cpp)
target_link_libraries(rs_BeliefStateCameraUpdater ${catkin_LIBRARIES} rs_BeliefStateCommunication ${unreal_interface_LIBRARIES})
rs_add_library(rs_BeliefStateObjectSynchronization src/BeliefStateObjectSynchronization.cpp)
target_link_libraries(rs_BeliefStateObjectSynchronization ${catkin_LIBRARIES} rs_BeliefStateCommunication rs_BeliefStateRepresentation)
rs_add_library(rs_BeliefStateObjectHighlighter src/BeliefStateObjectHighlighter.cpp)
target_link_libraries(rs_BeliefStateObjectHighlighter ${catkin_LIBRARIES} rs_BeliefStateRepresentation rs_BeliefStateCommunication)
rs_add_library(rs_BeliefStateSceneRefinement src/BeliefStateSceneRefinement.cpp)
target_link_libraries(rs_BeliefStateSceneRefinement ${catkin_LIBRARIES})
rs_add_library(rs_BeliefStateSensorPublisher src/BeliefStateSensorPublisher.cpp)
target_link_libraries(rs_BeliefStateSensorPublisher ${catkin_LIBRARIES} rs_BeliefStateCommunication)
rs_add_library(rs_CloudSimilarity src/CloudSimilarity.cpp)
target_link_libraries(rs_CloudSimilarity ${catkin_LIBRARIES} rs_featureTest)
rs_add_library(rs_featureTest src/featureTest.cpp)
target_link_libraries(rs_featureTest ${catkin_LIBRARIES})
