// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

// system includes
#include <iostream>
#include <string.h>

// Ace & Yarp includes
#include <yarp/os/Time.h>

// embObj  includes
#include "EOnv_hid.h"   // TODO needed??
#include <embObjSkin.h>

// Debug inlcludes
#ifdef _SETPOINT_TEST_
#include "EOYtheSystem.h"
#endif

using namespace std;

EmbObjSkin::EmbObjSkin() :  mutex(1)
{
    res         = NULL;
    ethManager  = NULL;
    initted     = false;
    sensorsNum  = 0;
    memset(info, 0x00, (size_t)SIZE_INFO);
};

bool EmbObjSkin::open(yarp::os::Searchable& config)
{
    std::string str;
    if(config.findGroup("GENERAL").find("Verbose").asInt())
        str=config.toString().c_str();
    else
        str="\n";
    yTrace() << str;

    // Tmp variables
    Bottle          groupEth, parameter;
//     ACE_TCHAR       address[64];
    int      port;

    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    sprintf(_fId.PC104ipAddr.string, "%s", parameter1.toString().c_str(), port);
    _fId.PC104ipAddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    sprintf(_fId.EMSipAddr.string, "%s", parameter2.toString().c_str());
    _fId.EMSipAddr.port = port;

    sscanf(_fId.EMSipAddr.string,"\"%d.%d.%d.%d", &_fId.EMSipAddr.ip1, &_fId.EMSipAddr.ip2, &_fId.EMSipAddr.ip3, &_fId.EMSipAddr.ip4);
    sscanf(_fId.PC104ipAddr.string,"\"%d.%d.%d.%d", &_fId.PC104ipAddr.ip1, &_fId.PC104ipAddr.ip2, &_fId.PC104ipAddr.ip3, &_fId.PC104ipAddr.ip4);

    sprintf(_fId.EMSipAddr.string,"%u.%u.%u.%u:%u", _fId.EMSipAddr.ip1, _fId.EMSipAddr.ip2, _fId.EMSipAddr.ip3, _fId.EMSipAddr.ip4, _fId.EMSipAddr.port);
    sprintf(_fId.PC104ipAddr.string,"%u.%u.%u.%u:%u", _fId.PC104ipAddr.ip1, _fId.PC104ipAddr.ip2, _fId.PC104ipAddr.ip3, _fId.PC104ipAddr.ip4, _fId.PC104ipAddr.port);

    // Check input parameters
    bool correct=true;

    sprintf(info, "EmbObjSkin - referred to EMS: %s", _fId.PC104ipAddr.string);

    if (!correct)
    {
        std::cerr<<"Error: insufficient parameters to EmbObjSkin\n";
        return false;
    }

    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "Unable to instantiate ethManager";
        return false;
    }

    Bottle ids=config.findGroup("SkinCanIds").tail();

    if (ids.size()>1)
    {
        yWarning() <<"EmbObjSkin id list contains more than one entry -> devices will be merged.";
    }
    for (int i=0; i<ids.size(); i++)
    {
        int id = ids.get(i).asInt();
        cardId.push_back (id);
    }

    //elements are:
    // sensorsNum=16*12*cardId.size();  // orig
    sensorsNum=16*12*7;		// max num of card
    data.resize(sensorsNum);

    int ttt = data.size();
    for (int i=0; i < ttt; i++)
        data[i]=(double)255;

    // fill FEAT_ID data
    _fId.ep = 255;
    _fId.type = Skin;
    std::string FeatId = config.find("FeatId").asString().c_str();
    strcpy(_fId.name, FeatId.c_str());

    _fId.boardNum  = 255;
    Value val =config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
    {
        _fId.boardNum =val.asInt();
        if((_fId.boardNum != 2) && (_fId.boardNum != 4))
        {
            yError() << "Requested non-existing Skin endpoint for board" << _fId.boardNum;
            return false;
        }
    }
    else
    {
        yError() << "skin: No board number found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        return false;
     }
    _fId.ep = eoprot_endpoint_skin;

    //N.B.: use a dynamic_cast to extract correct interface when using this pointer
    _fId.handle = (this);

    /* Once I'm ok, ask for resources, through the _fId struct I'll give the ip addr, port and
    *  and boradNum to the ethManagerin order to create the ethResource requested.
    * I'll Get back the very same sturct filled with other data useful for future handling
    * like the EPvector and EPhash_function */
    res = ethManager->requestResource(&_fId);
    if(NULL == res)
    {
        yError() << "EMS device not instantiated... unable to continue";
        return false;
    }
    init();

    res->goToRun();
    yTrace() << "EmbObj Skin for board " << _fId.boardNum << " correctly instatiated\n";
    return true;
}

bool EmbObjSkin::close()
{
    int ret = ethManager->releaseResource(_fId);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
    return true;
}

void EmbObjSkin::setId(FEAT_ID &id)
{
    _fId=id;
}

int EmbObjSkin::read(yarp::sig::Vector &out)
{
    mutex.wait();
    out=data;  //old - this needs the running thread
    mutex.post();

    return yarp::dev::IAnalogSensor::AS_OK;
}

int EmbObjSkin::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;;
}

int EmbObjSkin::getChannels()
{
    return sensorsNum;
}

int EmbObjSkin::calibrateSensor()
{
//#warning "create a ROP to start/initialize the MTB, if needed"

//	int 							j=0;
//	eOmc_joint_config_t				a;
//	uint16_t						sizze;
//
//	eOnvID_t nvid = eo_cfg_nvsEP_sk_NVID_Get(endpoint_sk_emsboard_leftlowerarm, 0x00, skinNVindex_sconfig__sigmode);
//	res->load_occasional_rop(eo_ropcode_set, endpoint_sk_emsboard_leftlowerarm, nvid);

    return true;
}

int EmbObjSkin::calibrateChannel(int ch, double v)
{
    //NOT YET IMPLEMENTED
    return calibrateSensor();
}

int EmbObjSkin::calibrateSensor(const yarp::sig::Vector& v)
{
    //data=v;
    return 0;
}

int EmbObjSkin::calibrateChannel(int ch)
{
    return 0;
}

bool EmbObjSkin::init()
{
//    char str[128];
    int j = 0;

    EOnv                        *cnv;
    eOmn_ropsigcfg_command_t    *ropsigcfgassign;
    EOarray                     *array;
    eOropSIGcfg_t               sigcfg;
    EOnv                        *nvRoot;
    EOnv                        nvtmp;

#ifdef _SETPOINT_TEST_
    eoy_sys_Initialise(NULL, NULL, NULL);
#endif

    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, 0, eoprot_tag_sk_skin_config_sigmode);

    uint8_t dat = 1;

   res->addSetMessage(protoid, &dat);

    //
    //	config regulars
    //
    EOnv nvtmp_ropsigcfgassign;
    eOprotID32_t protoid_ropsigcfgassign = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_ropsigcfg );

    cnv = res->getNVhandler(protoid_ropsigcfgassign, &nvtmp_ropsigcfgassign);
    ropsigcfgassign = (eOmn_ropsigcfg_command_t*) cnv->ram;
    array = (EOarray*) &ropsigcfgassign->array;
    eo_array_Reset(array);
    array->head.capacity = NUMOFROPSIGCFG;
    array->head.itemsize = sizeof(eOropSIGcfg_t);
    ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;

    protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_sk_skin, 0, eoprot_tag_sk_skin_status_arrayof10canframes);
    sigcfg.id32 = protoid;
    eo_array_PushBack(array, &sigcfg);

    // Send message
    if( !res->addSetMessage(protoid_ropsigcfgassign, (uint8_t *) ropsigcfgassign) )
    {
        yError() << "while setting rop sig cfg";
    }

    return true;
}

bool EmbObjSkin::fillData(void *raw_skin_data)
{
    uint8_t           msgtype = 0;
    uint8_t           i, triangle = 0;
    EOarray_of_10canframes 	*sk_array = (EOarray_of_10canframes*) raw_skin_data;
    static int error = 0;


    for(i=0; i<sk_array->head.size; i++)
    {
        eOutil_canframe_t *canframe;
        //uint8_t  j; 
        uint8_t mtbId =0;
        uint8_t  cardId, valid = 0;

        canframe = (eOutil_canframe_t*) &sk_array->data[i*sizeof(eOutil_canframe_t)];
        valid = (((canframe->id & 0x0F00) >> 8) == 3) ? 1 : 0;

        if(valid)
        {
            cardId = (canframe->id & 0x00f0) >> 4;
            switch (cardId)
            {
            case 14:
                mtbId = 0;
                break;
            case 13:
                mtbId = 1;
                break;
            case 12:
                mtbId = 2;
                break;
            case 11:
                mtbId = 3;
                break;
            case 10:
                mtbId = 4;
                break;
            case 9:
                mtbId = 5;
                break;
            case 8:
                mtbId = 6;
                break;
            default:
                yError() << "Unknown cardId from skin\n";
                return false;
                break;
            }
            triangle = (canframe->id & 0x000f);
            msgtype= ((canframe->data[0])& 0x80);

            int index=16*12*mtbId + triangle*12;

            if (msgtype)
            {
                for(int k=0; k<5; k++)
                {
                    data[index+k+7]=canframe->data[k+1];
                    // yError() << "fill data " << data[index+k+7];
                }
            }
            else
            {
                for(int k=0; k<7; k++)
                {
                    this->data[index+k]=canframe->data[k+1];
                    // yError() << "fill data " << data[index+k];
                }
            }
        }
        else if(canframe->id == 0x100)
        {
        	/* Can frame with id =0x100 contains Debug info. SO I skip it.*/
        	return true;
        }
        else
        {
        	if(error == 0)
        		yError() << "EMS: " << res->boardNum << " Unknown Message received from skin (" << i<<"/"<< sk_array->head.size<<"): frameID=" << canframe->id << " len="<<canframe->size << "data="<<canframe->data[0] << " " <<canframe->data[1] << " " <<canframe->data[2] << " " <<canframe->data[3] <<"\n" ;
        	error++;
        	if (error == 10000)
        		error = 0;
        }
    }
    return true;
}

// eof


