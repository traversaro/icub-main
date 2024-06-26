# Copyright (C) 2009 RobotCub Consortium
# Author: Alessandro Scalzo alessandro.scalzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

include(iCubUseQt5)

# Headers
set(iCubGui_SRCS animationview.cpp
                 bvh.cpp
                 bvhnode.cpp
                 camera.cpp
                 main.cpp
                 qavimator.cpp
                 settings.cpp
                 settingsdialog.cpp)

set(iCubGui_HDRS animationview.h
                 bvh.h
                 bvhnodedh.h
                 bvhnodeend.h
                 bvhnodeeye.h
                 bvhnodeforcetorque.h
                 bvhnode.h
                 bvhnodeinertial.h
                 bvhnodelefthand.h
                 bvhnoderighthand.h
                 bvhnoderoot.h
                 bvhnoderpy_xyz.h
                 camera.h
                 mesh.h
                 objectsthread.h
                 playstate.h
                 qavimator.h
                 settingsdialog.h
                 settings.h
                 subtitilessthread.h
                 visionobj.h)

set(iCubGui_QRC_FILES res.qrc)

set(iCubGui_UI_FILES qavimator.ui
                     settingsdialog.ui)


qt5_add_resources(iCubGui_QRC_GEN_SRCS ${iCubGui_QRC_FILES})
qt5_wrap_ui(iCubGui_UI_GEN_SRCS ${iCubGui_UI_FILES})

source_group("Source Files" FILES ${iCubGui_SRCS})
source_group("Header Files" FILES ${iCubGui_HDRS})
source_group("Resources Files" FILES ${iCubGui_QRC_FILES})
source_group("UI Files" FILES ${iCubGui_UI_FILES})
source_group("Generated Files" FILES ${iCubGui_QRC_GEN_SRCS}
                                     ${iCubGui_UI_GEN_SRCS})


include_directories(${Qt5_INCLUDE_DIRS})

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

add_executable(iCubGui WIN32 ${iCubGui_HDRS}
                             ${iCubGui_SRCS}
                             ${iCubGui_QRC_GEN_SRCS}
                             ${iCubGui_UI_GEN_SRCS})

target_compile_definitions(${PROJECT_NAME} PRIVATE ${QT_DEFINITIONS}
                                                   _USE_MATH_DEFINES
                                                   GLUT_NO_LIB_PRAGMA)

target_link_libraries(iCubGui GLUT::GLUT
                              OpenGL::GL
                              OpenGL::GLU
                              ${YARP_LIBRARIES}
                              skinDynLib)

qticub_use_modules(iCubGui Widgets
                           OpenGL)

if(WIN32 AND CMAKE_VERSION VERSION_LESS 2.8.11)
    target_link_libraries(iCubGui Qt5::WinMain)
endif()

# Install iCubGui
install(TARGETS iCubGui RUNTIME DESTINATION bin)
