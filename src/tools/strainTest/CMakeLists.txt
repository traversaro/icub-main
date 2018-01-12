# Copyright: (C) 2018 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(strainTest)

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})

include_directories(${YARP_INCLUDE_DIRS})

# TODO(traversaro) : this should be replaced by a proper use of target_include_directories
# but we should increase the CMake minimum version before
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../canLoader/canLoaderLib)

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} canLoaderLib)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)