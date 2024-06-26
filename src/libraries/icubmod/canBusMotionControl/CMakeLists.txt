# Copyright: (C) 2008 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(canmotioncontrol
    CATEGORY device
    TYPE yarp::dev::CanBusMotionControl
    INCLUDE CanBusMotionControl.h
    EXTRA_CONFIG WRAPPER=controlBoard_nws_yarp)

if (ENABLE_icubmod_canmotioncontrol)

if(ICUB_HAS_icub_firmware_shared)


   if (ICUB_CANPROTOCOL_STRICT)
       add_definitions(-DICUB_CANPROTOCOL_STRICT)
   endif()

   if (ICUB_CANMASKS_STRICT_FILTER)
       add_definitions(-DICUB_CANMASKS_STRICT_FILTER)
   endif()

   ADD_DEFINITIONS(-D_REENTRANT)

   INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} 
                       ../motionControlLib/)

   SET(folder_source CanBusMotionControl.cpp ThreadTable2.cpp ThreadPool2.cpp)
   SET(folder_header CanBusMotionControl.h ThreadTable2.h ThreadPool2.h)

   SOURCE_GROUP("Source Files" FILES ${folder_source})
   SOURCE_GROUP("Header Files" FILES ${folder_header})

   yarp_add_plugin(canmotioncontrol ${folder_source} ${folder_header})
   TARGET_LINK_LIBRARIES(canmotioncontrol ACE::ACE
                                          iCubDev
                                          YARP::YARP_os
                                          icub_firmware_shared::canProtocolLib)

   icub_export_plugin(canmotioncontrol)
            yarp_install(TARGETS canmotioncontrol
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

else(ICUB_HAS_icub_firmware_shared)
  message(ERROR " canBusMotionControl: cannot find icub_firmware_shared library, turn off device
  embObj library can now be found in the icub-firmware-shared package that
  can be downloaded from here:
  .
       https://github.com/robotology/icub-firmware-shared
  .    ")
endif(ICUB_HAS_icub_firmware_shared)

endif (ENABLE_icubmod_canmotioncontrol)


