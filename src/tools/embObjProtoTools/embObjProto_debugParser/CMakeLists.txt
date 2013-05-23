# Copyright: (C) 2010 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME embObjProto_debugParser)

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

set(EXTRA_FOLDER  		${CMAKE_CURRENT_SOURCE_DIR}/)


#headers-----------

#set(BASE 			$ENV{ICUB_ROOT}/firmware/emBODY/)
set(BASE 			$ENV{ICUB_ROOT}/main/src/libraries/icubmod/emBODY/)


set(CORE_FOLDER  		${BASE}any-arch/sys/embobj/core/src/)
set(CORE_API	  		${BASE}any-arch/sys/embobj/core/api/)
set(CORE_VEE_FOLDER  	${BASE}any-arch/sys/embobj/core-vee/src/)
set(PROT_FOLDER  		${BASE}any-arch/sys/embobj/prot/src/)
set(PORTING_FOLDER 		${BASE}porting/)
set(UTILS_FOLDER  		${BASE}any-arch/sys/embobj/utils/src/)
set(ICUB_API     		${BASE}any-arch/sys/embobj/icub/api/)


set(embObj_includes 	
						${CORE_API}
						${CORE_FOLDER}
						${PROT_FOLDER}../api/
						${PROT_FOLDER}
						${PORTING_FOLDER}
						${CORE_VEE_FOLDER}../api/
						${CORE_VEE_FOLDER}
						${UTILS_FOLDER}
						${UTILS_FOLDER}../api/
						${ICUB_API}
						)   

set(ALL_INCLUDES		${embObj_includes}
						${EXTRA_FOLDER})

include_directories (${ALL_INCLUDES} )
#message(INFO "includes: ${ALL_INCLUDES}")

#source code --------------

					
					


set(UTILS_SOURCE		${UTILS_FOLDER}/eODeb_eoProtoParser.c
				${UTILS_FOLDER}/eOtheEthLowLevelParser.c)

set(UTILS_HEADER		${UTILS_FOLDER}/eODeb_eoProtoParser_hid.h
				${UTILS_FOLDER}/eOtheEthLowLevelParser_hid.h)



set(CORE_SOURCE 		${CORE_FOLDER}/EOarray.c
				        ${CORE_FOLDER}/EOarray_hid.h
						${CORE_FOLDER}/EOtheErrorManager.c
						${CORE_FOLDER}/EOtheMemoryPool.c
						${CORE_FOLDER}/EoCommon.c
                        )

set(CORE_VEE_SOURCE			${CORE_VEE_FOLDER}/EOVmutex.c
							${CORE_VEE_FOLDER}/EOVtask.c
							${CORE_VEE_FOLDER}/EOVtheSystem.c	)

set(embobj_source 
                        ${CORE_SOURCE}						
                        ${UTILS_HEADER}
						${UTILS_SOURCE}
                        ${CORE_VEE_SOURCE})


#insert here name of parser cfg file!!! 
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_getPid.cpp )
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_searchPosSetpoint4all.cpp )
#set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_searchPosSetpoint.cpp )
set(PARSER_CFG_FILE    ${EXTRA_FOLDER}/parser_cfg_enaDisaJoints.cpp)

message(INFO "  you are using  ${PARSER_CFG_FILE} for parser cfg!!!")

set(EXTRA_SOURCE 	    ${PARSER_CFG_FILE}                
                        ${EXTRA_FOLDER}/main.cpp
						${EXTRA_FOLDER}/pcap_wrapper_linux.cpp
                        )

set(ALL_SOURCE_CODE				${embobj_source}
						${EXTRA_SOURCE})

add_executable(${PROJECTNAME} ${ALL_SOURCE_CODE})

target_link_libraries(${PROJECTNAME} pcap)
