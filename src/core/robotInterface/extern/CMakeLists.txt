# Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT


#if(ICUB_INTERFACE_EXPERIMENTAL)
    # TinyXML
    # if(YARP_USE_TINYXML AND NOT YARP_USE_SYSTEM_TINYXML)
        add_subdirectory(tinyxml)
    #endif(YARP_USE_TINYXML AND NOT YARP_USE_SYSTEM_TINYXML)
#endif()

