/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/* @file       eOcfg_nvsEP_mc_usrcbk_pippo.c
    @brief     This file keeps examples for ems / pc104 of the user-defined functions used for all endpoints in mc
    @author     marco.accame@iit.it
    @date       05/02/2012
**/

//#warning --> read this very important comment on the different use of group or general callbacks
// very important comment:
// the EOtheAgent calls a default callback which is specific of a given _usr group (upperarm, lowerarm, torso, upperleg, lowerleg).
// inside this callback it is possible to understand which endpoint is (left or right) by evaluating argument ep.
// the default group callback then calls the default motioncontrol callback. the same mechanism is used to understand which is the relevant
// endpoint.
// the two default callbacks do nothing else.
// the user can override one of the two by simply defining a new function with the same name and declaration.
// if the more specific group callback is redefined, then the more general is not called anymore for those groups which found this overriden.
// RULE TO USE: override only the general or the only the group ones. If the genral ones are overridden it is possible to write more general
//              code inside the ems.

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"

#include "EoCommon.h"
#include "EOnv_hid.h"
#include "EoProtocol.h"
#include "EoProtocolMC.h"

#include "EoProtocolMC_overridden_fun.h"
#include <FeatureInterface.h>
//#include "eOcfg_nvsEP_mc_hid.h"


#ifdef _ICUB_CALLBACK_
//#include "IRobotInterface.h"
#include "FeatureInterface.h"
#endif





#define DEG_2_ICUBDEG  182.04444

//void ep2char(char* str, uint16_t ep)
//{
//    switch(	ep)
//    {
//    case endpoint_mc_leftupperarm:
//        memcpy(str, "LA U MC\0", strlen("LA U MC\0")+1);
//        break;
//    case endpoint_mc_leftlowerarm:
//        memcpy(str, "LA L MC\0", strlen("LA L MC\0")+1);
//        break;
//    case endpoint_mc_rightupperarm:
//        memcpy(str, "RA U MC\0", strlen("RA U MC\0")+1);
//        break;
//    case endpoint_mc_rightlowerarm:
//        memcpy(str, "RA L MC\0", strlen("RA L MC\0")+1);
//        break;
//    case endpoint_mc_torso:
//        memcpy(str, "T MC\0", strlen("T MC\0")+1);
//        break;
//    case endpoint_mc_leftupperleg:
//        memcpy(str, "LL U MC\0", strlen("LL U MC\0")+1);
//        break;
//    case endpoint_mc_leftlowerleg:
//        memcpy(str, "LL L MC\0", strlen("LL L MC\0")+1);
//        break;
//    case endpoint_mc_rightupperleg:
//        memcpy(str, "RL U MC\0", strlen("RL U MC\0")+1);
//        break;
//    case endpoint_mc_rightlowerleg:
//        memcpy(str, "RL L MC\0", strlen("RL L MC\0")+1);
//        break;
//    }
//}

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions
// --------------------------------------------------------------------------------------------------------------------

void jwake(eOprotIndex_t xx, const EOnv* nv, eOprot_tag_mc_joint_t nv_name)
{
    eOprotID32_t protoid;
    void *handler = (void*) get_MChandler_fromEP(nvBoardNum2FeatIdBoardNum(eo_nv_GetBRD(nv)), eo_nv_GetEP8(nv));
    if(NULL == handler)
    {
        printf("eoMC class not found\n");
        return;
    }
    
    protoid = eo_nv_GetID32(nv);
    eOprotProgNumber_t  prognum = eoprot_endpoint_id2prognum(eo_nv_GetBRD(nv), protoid);
    MCmutex_post(handler, prognum);
}

////e'uguale a jwake
//void mwake(eOcfg_nvsEP_mc_motorNumber_t xx, const EOnv* nv,  eOcfg_nvsEP_mc_motorNVindex_t nv_name)
//{
//    void *handler = (void*) get_MChandler_fromEP(nv->ep);
//    eOnvID_t nvid = eo_cfg_nvsEP_mc_motor_NVID_Get(nv->ep, xx, nv_name);
//    uint16_t epindex, nvindex;
//    EP_NV_2_index(nv->ep, nvid, &epindex, &nvindex);
//    MCmutex_post(handler, epindex, nvindex);
//}

extern void eoprot_fun_UPDT_mc_joint_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{
//#warning "joint status basic strong iCubInterface"
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);
#define _debug_jstatus_basic_
#ifdef _debug_jstatus_basic_
    static int i = 0;
    static uint8_t print = 0;

    if( (1 == print) && (i>= 2000) && (xx == 0) )
    {
        i = 0;
        print = 0;
    }

    if(xx == 0)
        i++;

    if(i >= 2000)
    {
        print = 1;
    }

  //  if(print)
    {
        eOmc_joint_status_basic_t *jstatus_b = nv->ram;
        printf("\njstatus__basic for Joint num = %d\n", xx);
        printf("ep = 0x%X\n", eo_nv_GetEP8(nv));
        printf("jstatus_b->acceleration = 0x%X\n", jstatus_b->acceleration);
        printf("jstatus_b->controlmodestatus = 0x%X\n", jstatus_b->controlmodestatus);
        printf("jstatus_b->motionmonitorstatus = 0x%X\n", jstatus_b->motionmonitorstatus);
        printf("jstatus_b->position = 0x%X\n", jstatus_b->position);
        printf("jstatus_b->torque = 0x%X\n", jstatus_b->torque);
        printf("jstatus_b->velocity = 0x%X\n", jstatus_b->velocity);
    }
#endif

//	static int32_t zero = -360;
//	static FILE *positionLog = NULL;
//
//	if(NULL == positionLog)
//	{
//		positionLog = fopen("/usr/local/src/robot/debugging/position.log", "w");
//		if(NULL == positionLog)
//			printf("Error opening file /usr/local/src/robot/debugging/position.log!!!!!!!!!!!!!!!!!!!!!!\n");
//	}
//	else //(NULL != positionLog)
//	{
//		eOmc_joint_status_basic_t *jstatus_b = nv->rem;
//		if( (3 == xx) && (endpoint_mc_rightupperleg == nv->ep) )
//			fprintf(positionLog, "pos = 0x%X\n", ((jstatus_b->position - zero ) /DEG_2_ICUBDEG ) );
//	}
    {
    FEAT_ID id;
    int jointNum = (int) xx;

    id.ep = eo_nv_GetEP8(nv);
    id.type = MotionControl;
    id.boardNum =  eo_nv_GetBRD(nv);
    addEncoderTimeStamp( &id,  jointNum);
    }
}

extern void eoprot_fun_UPDT_mc_joint_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);
//#warning "joint status full strong iCubInterface"
//#define _debug_jstatus_full_
#ifdef  _debug_jstatus_full_
#define T 500

//	static int i = 0;
//	static bool print = false;
//	static int  j=-1;
//
//	if(-1 == j)
//		j = xx;
//
////	//transceiver_wait(nv->ep);
//	if( (print) && (i>= T) && (xx == j) )
//	{
//		i = 0;
//		print = false;
//		printf("\n");
//	}
//
//	if(xx == j)
//		i++;
//
//	if(i >= T)
//	{
//		print = true;
//	}
//
//	if(print)
//	{
//		eOmc_joint_status_t *jstatus = nv->rem;
//		printf("-->joint status full for joint num = %d\n", xx);
////		printf("ep = 0x%X\n", nv->ep);
//		printf("jStatus->mode     = 0x%X\n", jstatus->basic.controlmodestatus);
////		printf("jStatus->pid->Ref = 0x%X\n", jstatus->ofpid.reference);
//		printf("jStatus->position = 0x%X\n", jstatus->basic.position);
////		printf("jStatus->velocity = 0x%X\n", jstatus->basic.velocity);
//	}
//	//transceiver_post(nv->ep);

#endif

//	static int32_t zero = 360;			// dpvrebbe essere  -360, ma pare che qui "X - (-360)" sia diverso da " X+360"
//	static FILE *positionLog = NULL;
//	static bool init = false;
//	int32_t pos = -1;
//
//	if(NULL == positionLog)
//	{
//		if(false == init)
//		{
//			positionLog = fopen("/usr/local/src/robot/debugging/position.log", "w");
//			if(NULL == positionLog)
//				printf("Error opening file /usr/local/src/robot/debugging/position.log\n");
//			else
//				printf("positionLog file opened succesfully!!\n");
//		}
//	}
//	else
//	{
//		eOmc_joint_status_basic_t *jstatus_b = nv->rem;
//		if( (3 == xx) && (endpoint_mc_rightupperleg == nv->ep) )
//		{
//			pos = (jstatus_b->position / DEG_2_ICUBDEG) + zero;
////			fprintf(positionLog, "%d\n", pos);
//		}
//	}
//	init = true;

    // Add timestamp
    FEAT_ID id;

    int jointNum = (int) xx;

    id.ep = eo_nv_GetEP8(nv);
    id.type = MotionControl;
    id.boardNum =  nvBoardNum2FeatIdBoardNum(eo_nv_GetBRD(nv));
    addEncoderTimeStamp( &id,  jointNum);

#ifdef _SETPOINT_TEST_
    //per test

    static
    setpoint_test_data_t *rec_test_data_ptr;
    eOmc_joint_status_t *jstatus = nv->rem;
    uint8_t *aux = (uint8_t*)jstatus;
    rec_test_data_ptr = (setpoint_test_data_t *)&aux[16];

    check_received_debug_data(&id, jointNum, rec_test_data_ptr);
#endif
}

extern void eoprot_fun_UPDT_mc_motor_status_basic(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);
//#warning "motor status basic strong iCubInterface"
//#define _debug_mstatus_basic_
#ifdef _debug_mstatus_basic_
    static int i = 0;
    static bool print = false;

    //transceiver_wait(nv->ep);

    if( (print) && (i>= 2000) && (xx == 0) )
    {
        i = 0;
        print = false;
    }

    if(xx == 0)
        i++;

    if(i >= 2000)
    {
        print = true;
    }

    if(print)
    {
        eOmc_motor_status_basic_t *mstatus_b = nv->rem;
//		printf("mstatus__basic for motor num = %d\n", xx);
//		printf("ep = 0x%X\n", nv->ep);
//		printf("mstatus_b->current  = 0x%X\n", mstatus_b->current);
//		printf("mstatus_b->filler02 = 0x%X\n", mstatus_b->filler02);
        printf("mstatus_b->position = 0x%X\n", mstatus_b->position);
//		printf("mstatus_b->velocity = 0x%X\n", mstatus_b->velocity);
    }
    //transceiver_post(nv->ep);
#endif
}

extern void eoprot_fun_UPDT_mc_joint_config(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);
#ifdef _debug_jxx_jconfig_
    //transceiver_wait(nv->ep);
    eOmc_joint_config_t *jConfig = nv->rem;
    printf("\nmaxpositionofjoint for Joint num = %d\n", xx);
    printf("jConfig->pidposition.kp 	= 0x%X\n",	jConfig->pidposition.kp		);
    printf("jConfig->pidposition.ki 	= 0x%X\n",	jConfig->pidposition.ki		);
    printf("jConfig->pidposition.kd 	= 0x%X\n",		jConfig->pidposition.kd);
    printf("jConfig->pidposition.limitonintegral = 0x%X\n",	jConfig->pidposition.limitonintegral);
    printf("jConfig->pidposition.limitonoutput = 0x%X\n",	jConfig->pidposition.limitonoutput);
    printf("jConfig->pidposition.offset 	= 0x%X\n",		jConfig->pidposition.offset);
    printf("jConfig->pidposition.scale 		= 0x%X\n",	jConfig->pidposition.scale);
    printf("jConfig->minpositionofjoint		= 0x%X\n",	jConfig->minpositionofjoint);
    printf("jConfig->maxpositionofjoint		= 0x%X\n",	jConfig->maxpositionofjoint);
    printf("jConfig->controlmode			= 0x%X\n",	jConfig->motionmonitormode);
    printf("ep = 0x%X\n", nv->ep);
    //transceiver_post(nv->ep);
#endif
}

extern void eoprot_fun_UPDT_mc_motor_config(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOprotIndex_t xx = eoprot_ID2index(rd->id32);
//#warning "mconfig strong iCubInterface"
//#define _debug_mxx_mconfig_
#ifdef _debug_mxx_mconfig_
    //transceiver_wait(nv->ep);
    eOmc_motor_config_t *jConfig = nv->rem;
    printf("\nmaxpositionofjoint for Joint num = %d\n", xx);
    printf("mConfig->pidcurrent.kp 	= 0x%X\n",	jConfig->pidcurrent.kp		);
    printf("mConfig->pidcurrent.ki 	= 0x%X\n",	jConfig->pidcurrent.ki		);
    printf("mConfig->pidcurrent.kd 	= 0x%X\n",		jConfig->pidcurrent.kd);
    printf("mConfig->pidcurrent.limitonintegral = 0x%X\n",	jConfig->pidcurrent.limitonintegral);
    printf("mConfig->pidcurrent.limitonoutput = 0x%X\n",	jConfig->pidcurrent.limitonoutput);
    printf("mConfig->pidcurrent.offset 	= 0x%X\n",		jConfig->pidcurrent.offset);
    printf("mConfig->pidcurrent.scale 		= 0x%X\n",	jConfig->pidcurrent.scale);
    printf("mConfig->maxvelocityofmotor		= 0x%X\n",	jConfig->maxvelocityofmotor);
    printf("mConfig->maxcurrentofmotor		= 0x%X\n",	jConfig->maxcurrentofmotor);
    printf("ep = 0x%X\n", nv->ep);
    //transceiver_post(nv->ep);
#endif
}



extern void eoprot_fun_UPDT_mc_joint_config_limitsofjoint(const EOnv* nv, const eOropdescriptor_t* rd)
{
    jwake(eoprot_ID2index(rd->id32), nv, eoprot_tag_mc_joint_config_limitsofjoint);
}

extern void eoprot_fun_UPDT_mc_motor_config_maxcurrentofmotor(const EOnv* nv, const eOropdescriptor_t* rd)
{
//#warning "maxcurrentofmoto strong iCubInterface"
    //transceiver_wait(nv->ep);
    eOmeas_position_t *jMaxCurrent_b = nv->ram;
//    printf("\ncallback: maxcurrentofmotor for Joint num = %d\n", xx);
//    printf("ep = 0x%X\n", nv->ep);
//    printf("maxcurrentofmotor = 0x%X\n", jMaxCurrent_b);
    //transceiver_post(nv->ep);
}

extern void eoprot_fun_UPDT_mc_joint_config_pidposition(const EOnv* nv, const eOropdescriptor_t* rd)
{
//#warning "Position Pid strong iCubInterface"
//    char str[16];
//    ep2char(str, nv->ep);
 //   printf("Position Pid Callback ep =0x%0X(%s), j=%d (0x%0X) \n", nv->ep, str, xx, xx);
    //yDebug()<< "callback pid position: nvid=" << rd->id32 << "progressivenum=" << eoprot_ID2index(rd->id32);
    jwake(eoprot_ID2index(rd->id32), nv, eoprot_tag_mc_joint_config_pidposition);
}

extern void eoprot_fun_UPDT_mc_joint_config_pidtorque(const EOnv* nv, const eOropdescriptor_t* rd)
{
//#warning "Torque Pid strong iCubInterface"
//    char str[16];
//    ep2char(str, nv->ep);
//    printf("Torque Pid Callback ep =0x%0X(%s), j=%d (0x%0X) \n", nv->ep, str, xx, xx);
    jwake(eoprot_ID2index(rd->id32), nv, eoprot_tag_mc_joint_config_pidtorque);
}

//extern void eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jconfig__controlmode(const EOnv* nv, const eOropdescriptor_t* rd)
//{
////    printf("jconfig__controlmode, j=%d\n", xx);
//    //transceiver_wait(nv->ep);
//    //jwake(xx, nv, jointNVindex_jcmmnds__controlmode);	//???
//    //transceiver_post(nv->ep);
//}

extern void eoprot_fun_UPDT_mc_joint_cmmnds_setpoint(const EOnv* nv, const eOropdescriptor_t* rd)
{
//#warning "joint jcmmnds setpoint strong iCubInterface"

    eOmc_setpoint_t *setpoint_got = (eOmc_setpoint_t*) nv->ram;
    static int32_t zero = 360;
    static prev = 0;
    int32_t pos;

//	printf("Callback recv setpoint: \n");
    uint16_t *checkProg = (uint16_t*) setpoint_got;
//	printf("Prog Num %d\n", checkProg[1]);
    pos = (int32_t)(setpoint_got->to.position.value / DEG_2_ICUBDEG) + zero;
//	printf("position %d\n", pos);
//	printf("velocity %d\n", setpoint_got->to.position.withvelocity);

    if( (checkProg[1] - prev) != 1)
        printf(">>>>>>Missing packet!! prev was %d, actual is %d (missing 0x%04X) <<<<<\n", prev, checkProg[1], prev+1);

    prev = checkProg[1];
    jwake(eoprot_ID2index(rd->id32), nv, eoprot_tag_mc_joint_cmmnds_setpoint);
}

extern void eoprot_fun_UPDT_mc_joint_config_impedance(const EOnv* nv, const eOropdescriptor_t* rd)
{
////#warning "joint jconfig impedance strong iCubInterface"
//    char str[16];
//    ep2char(str, nv->ep);
////    printf("Impedance Params Callback ep =0x%0X(%s), j=%d (0x%0X) \n", nv->ep, str, j, j);
    jwake(eoprot_ID2index(rd->id32), nv, eoprot_tag_mc_joint_config_impedance);
}

// eof


