	// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Alberto Cardellino
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

//#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <string.h>
#include <iostream>

#include "embObjMotionControl.h"
#include <ethManager.h>
#include <FeatureInterface.h>

#ifdef _SETPOINT_TEST_
#include "EOYtheSystem.h"
#endif
#include "Debug.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

//#warning "Macro EMS_capacityofropframeregulars defined by hand!! Find a way to have this number synchronized with EMS!!"
#define EMS_capacityofropframeregulars 1204

#define MAXNUMOFJOINTS 16

// Utilities

static void copyPid_iCub2eo(const Pid *in, eOmc_PID_t *out)
{
    out->kp = (int16_t)in->kp;
    out->ki = (int16_t)in->ki;
    out->kd = (int16_t)in->kd;
    out->limitonintegral = (int16_t)in->max_int;
    out->limitonoutput = (int16_t)in->max_output;
    out->offset = (int16_t)in->offset;
    out->scale = (int8_t)in->scale;
}

static void copyPid_eo2iCub(eOmc_PID_t *in, Pid *out)
{
    out->kp = in->kp;
    out->ki = in->ki;
    out->kd = in->kd;
    out->max_int = in->limitonintegral;
    out->max_output = in->limitonoutput;
    out->offset = in->offset;
    out->scale = in->scale;
}

// This will be moved in the ImplXXXInterface
static double convertA2I(double angle_in_degrees, double zero, double factor)
{
    return (angle_in_degrees + zero) * factor;
}

static inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    yError() << txt << "is not yet implemented for embObjMotionControl";
    return true;
}

#define NV_NOT_FOUND	return nv_not_found();

static bool nv_not_found(void)
{
    yError () << " nv_not_found!! This may mean that this variable is not handled by this EMS\n";
    return false;
}


//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool embObjMotionControl::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError () << key1.c_str() << " not found\n";
        return false;
    }

    if(tmp.size()!=size)
    {
        yError () << key1.c_str() << " incorrect number of entries in board " << _fId.name << '[' << _fId.boardNum << ']';
        return false;
    }

    out=tmp;
    return true;
}


bool embObjMotionControl::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
    _encodersStamp = allocAndCheck<double>(nj);
    _encoderconversionoffset = allocAndCheck<float>(nj);
    _encoderconversionfactor = allocAndCheck<float>(nj);

    _rotToEncoder = allocAndCheck<double>(nj);
    _zeros = allocAndCheck<double>(nj);
    _torqueSensorId= allocAndCheck<int>(nj);
    _torqueSensorChan= allocAndCheck<int>(nj);
    _maxTorque=allocAndCheck<double>(nj);
    _newtonsToSensor=allocAndCheck<double>(nj);

    _pids=allocAndCheck<Pid>(nj);
    _tpids=allocAndCheck<Pid>(nj);

    _impedance_params=allocAndCheck<ImpedanceParameters>(nj);
    _impedance_limits=allocAndCheck<ImpedanceLimits>(nj);
    _estim_params=allocAndCheck<SpeedEstimationParameters>(nj);

    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _currentLimits=allocAndCheck<double>(nj);
    checking_motiondone=allocAndCheck<bool>(nj);

    _velocityShifts=allocAndCheck<int>(nj);
    _velocityTimeout=allocAndCheck<int>(nj);

    // Reserve space for data stored locally. values are initialize to 0
    _ref_positions = allocAndCheck<double>(nj);
    _command_speeds = allocAndCheck<double>(nj);
    _ref_speeds = allocAndCheck<double>(nj);
    _ref_accs = allocAndCheck<double>(nj);
    _ref_torques = allocAndCheck<double>(nj);
    _enabledAmp = allocAndCheck<bool>(nj);
    _enabledPid = allocAndCheck<bool>(nj);
    _calibrated = allocAndCheck<bool>(nj);
    _cacheImpedance = allocAndCheck<eOmc_impedance_t>(nj);

    //debug purpose

    
#ifdef VERIFY_ROP_SETIMPEDANCE 
    impedanceSignature = allocAndCheck<uint32_t> (nj);
#endif

#ifdef VERIFY_ROP_SETPOSITIONRAW
    refRawSignature = allocAndCheck<uint32_t> (nj);
    sendingDirects = allocAndCheck<bool>(nj);
#endif

#ifdef _SETPOINT_TEST_
    j_debug_data = allocAndCheck<debug_data_of_joint_t>(nj);
#endif
    //	_debug_params=allocAndCheck<DebugParameters>(nj);


    return true;
}

bool embObjMotionControl::dealloc()
{
    delete _axisMap;
    delete _angleToEncoder;
    delete _encodersStamp;
    delete _encoderconversionoffset;
    delete _encoderconversionfactor;

    delete _rotToEncoder;
    delete _zeros;
    delete _torqueSensorId;
    delete _torqueSensorChan;
    delete _maxTorque;
    delete _newtonsToSensor;

//    if(NULL != _pids);
//        delete _pids;
//    delete _tpids;

    delete _impedance_params;
    delete _impedance_limits;
    delete _estim_params;

    delete _limitsMax;
    delete _limitsMin;
    delete _currentLimits;
    delete checking_motiondone;

    delete _velocityShifts;
    delete _velocityTimeout;

    // Reserve space for data stored locally. values are initialize to 0
    delete _ref_positions;
    delete _command_speeds;
    delete _ref_speeds;
    delete _ref_accs;
    delete _ref_torques;
    delete _enabledAmp;
    delete _enabledPid;
    delete _calibrated;

#ifdef _SETPOINT_TEST_
    delete j_debug_data;
#endif
    //  _debug_params=allocAndCheck<DebugParameters>(nj);


    return true;
}

embObjMotionControl::embObjMotionControl() :
    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>(this),
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>(this),
    ImplementPidControl<embObjMotionControl, IPidControl>(this),
    ImplementEncodersTimed(this),
    ImplementPositionControl2(this),
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>(this),
    ImplementVelocityControl2(this),
    ImplementControlMode(this),
    ImplementImpedanceControl(this),
#ifdef IMPLEMENT_DEBUG_INTERFACE
    ImplementDebugInterface(this),
#endif
    ImplementTorqueControl(this),
    ImplementControlLimits2(this),
    ImplementPositionDirect(this),
    _mutex(1),
    SAFETY_THRESHOLD(2.0)
{
    initted       = 0;
    _pids         = NULL;
    _tpids        = NULL;
    res           = NULL;
    requestQueue  = NULL;
    _tpidsEnabled = false;
    _njoints      = 0;
    _axisMap      = NULL;
    _zeros        = NULL;

    _encoderconversionfactor = NULL;
    _encoderconversionoffset = NULL;
    _angleToEncoder = NULL;

    _cacheImpedance   = NULL;
    _impedance_params = NULL;
    _impedance_limits = NULL;
    _estim_params     = NULL;

    _limitsMin        = NULL;
    _limitsMax        = NULL;
    _currentLimits    = NULL;
    _velocityShifts   = NULL;
    _velocityTimeout  = NULL;
    _torqueSensorId   = NULL;
    _torqueSensorChan = NULL;
    _maxTorque        = NULL;
    _newtonsToSensor  = NULL;
    _rotToEncoder     = NULL;
    _ref_accs         = NULL;
    _command_speeds   = NULL;
    _ref_positions    = NULL;
    _ref_speeds       = NULL;
    _ref_torques      = NULL;

    checking_motiondone = NULL;
    // debug connection
    tot_packet_recv   = 0;
    errors            = 0;
    start             = 0;
    end               = 0;

    // Check status of joints
    _enabledPid       = NULL;
    _enabledAmp       = NULL;
    _calibrated       = NULL;
    // NV stuff
    NVnumber          = 0;

    useRawEncoderData = false;
}

embObjMotionControl::~embObjMotionControl()
{
    dealloc();
    //YARP_INFO(Logger::get(),"embObjMotionControl::~embObjMotionControl()", Logger::get().log_files.f3);
    /*if (handle!=0)
    {
    	sprintf(tmp, "embObjMotionControl::~embObjMotionControl() 2 handle= 0x%06X", handle);
    	//YARP_DEBUG(Logger::get(),tmp); //, Logger::get().log_files.f3);
    		delete handle;
    }*/
}

bool embObjMotionControl::open(yarp::os::Searchable &config)
{
    std::string str;
    if(!config.findGroup("GENERAL").find("verbose").isBool())
    {
        yWarning() << " general->verbose bool param is different from accepted values (true / false). Assuming false";
        str=" ";
    }
    else
    {
        if(config.findGroup("GENERAL").find("verbose").asBool())
            str=config.toString().c_str();
        else
            str=" ";
    }
    yTrace() << str;

    // Tmp variables
    Bottle          groupEth;
    ACE_UINT16      port;

    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    if(groupEth.isNull())
    {
        yError() << "Can't find ETH group in config files";
        return false;
    }
    Value *PC104IpAddress_p;
    if(!groupEth.check("PC104IpAddress", PC104IpAddress_p))
    {
        yError() << "missing PC104IpAddress";
        return false;
    }
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    strcpy(_fId.PC104ipAddr.string, parameter1.toString().c_str());
    _fId.PC104ipAddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    strcpy(_fId.EMSipAddr.string, parameter2.toString().c_str());
    _fId.EMSipAddr.port = port;

    sscanf(_fId.EMSipAddr.string,"\"%d.%d.%d.%d", &_fId.EMSipAddr.ip1, &_fId.EMSipAddr.ip2, &_fId.EMSipAddr.ip3, &_fId.EMSipAddr.ip4);
    sscanf(_fId.PC104ipAddr.string,"\"%d.%d.%d.%d", &_fId.PC104ipAddr.ip1, &_fId.PC104ipAddr.ip2, &_fId.PC104ipAddr.ip3, &_fId.PC104ipAddr.ip4);

    sprintf(_fId.EMSipAddr.string,"%u.%u.%u.%u:%u", _fId.EMSipAddr.ip1, _fId.EMSipAddr.ip2, _fId.EMSipAddr.ip3, _fId.EMSipAddr.ip4, _fId.EMSipAddr.port);
    sprintf(_fId.PC104ipAddr.string,"%u.%u.%u.%u:%u", _fId.PC104ipAddr.ip1, _fId.PC104ipAddr.ip2, _fId.PC104ipAddr.ip3, _fId.PC104ipAddr.ip4, _fId.PC104ipAddr.port);

    // Check input parameters
    sprintf(info, "embObjMotionControl - referred to EMS: %s", _fId.EMSipAddr.string);

    // Check useRawEncoderData = do not use calibration data!
    Value use_raw = config.findGroup("GENERAL").find("useRawEncoderData");

    if(use_raw.isNull())
    {
        useRawEncoderData = false;
    }
    else
    {
        if(!use_raw.isBool())
        {
            yWarning() << " useRawEncoderData bool param is different from accepted values (true / false). Assuming false";
            useRawEncoderData = false;
        }
        else
        {
            useRawEncoderData = use_raw.asBool();
            if(useRawEncoderData)
                yWarning() << "eo MotionControl using raw data from encoders! Be careful. \n" <<
                              "\t  DO NOT USE OR CALIBRATE THE ROBOT IN THIS CONFIGURATION! See 'useRawEncoderData' param in config file";
        }
    }

//    yWarning() << "useRawEncoderData is " << useRawEncoderData;

    // Saving User Friendly Id
    memset(_fId.name, 0x00, SIZE_INFO);
    sprintf(_fId.name, "%s", info);

    _fId.boardNum  = 255;
    Value val =config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
        _fId.boardNum =val.asInt();
    else
    {
        yError () << "embObjMotionControl: EMS Board number identifier not found";
        return false;
    }

    if(_fId.boardNum > 9)
    {
        _fId.ep = 255;
        yError () << "\n embObjMotionControl: board with number "<<_fId.boardNum << "not -exists!!!" ;
        return false;
    }

    _fId.ep = eoprot_endpoint_motioncontrol;
    //
    //  Read Configuration params from file
    //
    _njoints = config.findGroup("GENERAL").check("Joints",Value(1),   "Number of degrees of freedom").asInt();

    if(!alloc(_njoints))
    {
        yError() << "Malloc failed";
        return false;
    }
    if(!fromConfig(config))
    {
        yError() << "Missing parameters in config file";
        return false;
    }

    //  INIT ALL INTERFACES

    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPositionControl2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementPidControl<embObjMotionControl, IPidControl>:: initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementControlMode::initialize(_njoints, _axisMap);
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementVelocityControl2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
#ifdef IMPLEMENT_DEBUG_INTERFACE
    ImplementDebugInterface::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _rotToEncoder);
#endif
    ImplementControlLimits2::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);
    ImplementImpedanceControl::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _newtonsToSensor);
    ImplementTorqueControl::initialize(_njoints, _axisMap, _angleToEncoder, _zeros, _newtonsToSensor);
    ImplementPositionDirect::initialize(_njoints, _axisMap, _angleToEncoder, _zeros);

    /*
    *  Once I'm sure every input data required is present and correct, instantiate the EMS, transceiver etc...
    */

    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "Unable to instantiate ethManager";
        return false;
    }

    _fId.handle  = (this);
    res = ethManager->requestResource(&_fId);
    if(NULL == res)
    {
        yError() << "EMS device not instantiated... unable to continue";
        return false;
    }

    NVnumber = res->getNVnumber(_fId.boardNum, _fId.ep);
    requestQueue = new eoRequestsQueue(NVnumber);

    if(!init() )
    {
        yError() << "while initing board " << _fId.boardNum;
        return false;
    }

    if(!configure_mais())
    {
        yError() << "while configuring mais for board " << _fId.boardNum;
        return false;
    }

    initted = true;
    //Time::delay(2);
    res->goToRun();
    return true;
}


bool embObjMotionControl::fromConfig(yarp::os::Searchable &config)
{
    Bottle xtmp;
    int i;
    Bottle general = config.findGroup("GENERAL");

    // leggere i valori da file
    if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints))
        return false;

    for (i = 1; i < xtmp.size(); i++)
        _axisMap[i-1] = xtmp.get(i).asInt();

    double tmp_A2E;
    // Encoder scales
    if (!extractGroup(general, xtmp, "Encoder", "a list of scales for the encoders", _njoints))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
        {
            tmp_A2E = xtmp.get(i).asDouble();

            if(useRawEncoderData)   // do not use any configuration, this is intended for doing the very first calibration
            {
            	tmp_A2E > 0 ? _encoderconversionfactor[i-1] = 1 : _encoderconversionfactor[i-1] = -1;
                _angleToEncoder[i-1] = 1;
            }
            else
            {
                _angleToEncoder[i-1] =  (1<<16) / 360.0;		// conversion factor from degrees to iCubDegrees
                _encoderconversionfactor[i-1] = float((tmp_A2E  ) / _angleToEncoder[i-1]);
            }

            _encoderconversionoffset[i-1] = 0;
        }


    // Rotor scales
    if (!extractGroup(general, xtmp, "Rotor", "a list of scales for the rotor encoders", _njoints))
    {
        fprintf(stderr, "Using default value = 1\n");
        for(i=1; i<_njoints+1; i++)
            _rotToEncoder[i-1] = 1.0;
    }
    else
    {
        int test = xtmp.size();
        for (i = 1; i < xtmp.size(); i++)
            _rotToEncoder[i-1] = xtmp.get(i).asDouble();
    }

    // Zero Values
    if (!extractGroup(general, xtmp, "Zeros","a list of offsets for the zero point", _njoints))
        return false;
    else
        for (i = 1; i < xtmp.size(); i++)
            if(useRawEncoderData)   // do not use any configuration, this is intended for doing the very first calibration
                _zeros[i-1] = 0;
            else
            	_zeros[i-1] = xtmp.get(i).asDouble();


    // Torque Id
    if (!extractGroup(general, xtmp, "TorqueId","a list of associated joint torque sensor ids", _njoints))
    {
        fprintf(stderr, "Using default value = 0 (disabled)\n");
        for(i=1; i<_njoints+1; i++)
            _torqueSensorId[i-1] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorId[i-1] = xtmp.get(i).asInt();
    }


    if (!extractGroup(general, xtmp, "TorqueChan","a list of associated joint torque sensor channels", _njoints))
    {
        fprintf(stderr, "Using default value = 0 (disabled)\n");
        for(i=1; i<_njoints+1; i++)
            _torqueSensorChan[i-1] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorChan[i-1] = xtmp.get(i).asInt();
    }


    if (!extractGroup(general, xtmp, "TorqueMax","full scale value for a joint torque sensor", _njoints))
    {
        fprintf(stderr, "Using default value = 0\n");
        for(i=1; i<_njoints+1; i++)
        {
            _maxTorque[i-1] = 0;
            _newtonsToSensor[i-1]=1;
        }
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
        {
            _maxTorque[i-1] = xtmp.get(i).asInt();
            _newtonsToSensor[i-1] = double(0x8000)/double(_maxTorque[i-1]);
        }
    }

    ////// PIDS
    Bottle pidsGroup=config.findGroup("PIDS", "PID parameters");
    if (pidsGroup.isNull()) 
    {
        yError() << ": no PIDS group found in for board" << _fId.boardNum << "... closing";
        return false;
    }

    int j=0;
    for(j=0; j<_njoints; j++)
    {
        char tmp[80];
        sprintf(tmp, "Pid%d", j);

        Bottle &xtmp2 = pidsGroup.findGroup(tmp);
        _pids[j].kp = xtmp2.get(1).asDouble();
        _pids[j].kd = xtmp2.get(2).asDouble();
        _pids[j].ki = xtmp2.get(3).asDouble();

        _pids[j].max_int = xtmp2.get(4).asDouble();
        _pids[j].max_output = xtmp2.get(5).asDouble();

        _pids[j].scale = xtmp2.get(6).asDouble();
        _pids[j].offset = xtmp2.get(7).asDouble();
    }


    ////// TORQUE PIDS
    Bottle TPidsGroup=config.findGroup("TORQUE_PIDS","TORQUE_PID parameters");
    if (TPidsGroup.isNull())
    {
        yWarning( ) << "no TORQUE PIDS group found for board" << _fId.boardNum << ", skipping";
    }
    else
    {
        //yDebug( ) << "TORQUE PIDS group found for board" << _fId.boardNum;
        _tpidsEnabled=true;
        for(j=0; j<_njoints; j++)
        {
            char str1[80];
            sprintf(str1, "TPid%d", j);

            Bottle &xtmp3 = TPidsGroup.findGroup(str1);
            _tpids[j].kp = xtmp3.get(1).asDouble();
            _tpids[j].kd = xtmp3.get(2).asDouble();
            _tpids[j].ki = xtmp3.get(3).asDouble();

            _tpids[j].max_int = xtmp3.get(4).asDouble();
            _tpids[j].max_output = xtmp3.get(5).asDouble();

            _tpids[j].scale = xtmp3.get(6).asDouble();
            _tpids[j].offset = xtmp3.get(7).asDouble();
        }
    }


    if (!extractGroup(general, xtmp, "TorqueChan","a list of associated joint torque sensor channels", _njoints))
    {
        yWarning() <<  "Using default value = 0 (disabled)";
        for(i=1; i<_njoints+1; i++)
            _torqueSensorChan[i-1] = 0;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorChan[i-1] = xtmp.get(i).asInt();
    }

    if (general.check("IMPEDANCE","DEFAULT IMPEDANCE parameters")==true)
    {
        yWarning() << "IMPEDANCE parameters section found";
        for(j=0; j<_njoints; j++)
        {
            char str2[80];
            sprintf(str2, "Imp%d", j);
            if (config.findGroup("IMPEDANCE","DEFAULT IMPEDANCE parameters").check(str2)==true)
            {
                xtmp = config.findGroup("IMPEDANCE","DEFAULT IMPEDANCE parameters").findGroup(str2);
                _impedance_params[j].enabled=true;
                _impedance_params[j].stiffness = xtmp.get(1).asDouble();
                _impedance_params[j].damping   = xtmp.get(2).asDouble();
            }
        }
    }
    else
    {
        yWarning() << "Impedance section NOT enabled, skipping.";
    }

    ////// IMPEDANCE LIMITS DEFAULT VALUES (UNDER TESTING)
    for(j=0; j<_njoints; j++)
    {
        // got from canBusMotionControl, ask to Randazzo Marco
        _impedance_limits[j].min_damp=  0.001;
        _impedance_limits[j].max_damp=  9.888;
        _impedance_limits[j].min_stiff= 0.002;
        _impedance_limits[j].max_stiff= 9.889;
        _impedance_limits[j].param_a=   0.011;
        _impedance_limits[j].param_b=   0.012;
        _impedance_limits[j].param_c=   0.013;
    }

    /////// LIMITS
    Bottle &limits=config.findGroup("LIMITS");
    if (limits.isNull())
    {
        yWarning() << "Group LIMITS not found in configuration file";
        return false;
    }
    // current limit
    if (!extractGroup(limits, xtmp, "Currents","a list of current limits", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _currentLimits[i-1]=xtmp.get(i).asDouble();

    // max limit
    if (!extractGroup(limits, xtmp, "Max","a list of maximum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _limitsMax[i-1]=xtmp.get(i).asDouble();

    // min limit
    if (!extractGroup(limits, xtmp, "Min","a list of minimum angles (in degrees)", _njoints))
        return false;
    else
        for(i=1; i<xtmp.size(); i++) _limitsMin[i-1]=xtmp.get(i).asDouble();

    /////// [VELOCITY]
    Bottle &velocityGroup=config.findGroup("VELOCITY");
    if (!velocityGroup.isNull())
    {
        /////// Shifts
        if (!extractGroup(velocityGroup, xtmp, "Shifts", "a list of shifts to be used in the vmo control", _njoints))
        {
            fprintf(stderr, "Using default Shifts=4\n");
            for(i=1; i<_njoints+1; i++)
                _velocityShifts[i-1] = 4;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _velocityShifts[i-1]=xtmp.get(i).asInt();
        }

        /////// Timeout
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "Timeout", "a list of timeout to be used in the vmo control", _njoints))
        {
            fprintf(stderr, "Using default Timeout=100, i.e 0.1s\n");
            for(i=1; i<_njoints+1; i++)
                _velocityTimeout[i-1] = 100;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _velocityTimeout[i-1]=xtmp.get(i).asInt();
        }

        /////// Joint Speed Estimation
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "JNT_speed_estimation", "a list of shift factors used by the firmware joint speed estimator", _njoints))
        {
            fprintf(stderr, "Using default value=5\n");
            for(i=1; i<_njoints+1; i++)
                _estim_params[i-1].jnt_Vel_estimator_shift = 0;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _estim_params[i-1].jnt_Vel_estimator_shift = xtmp.get(i).asInt();
        }

        /////// Motor Speed Estimation
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "MOT_speed_estimation", "a list of shift factors used by the firmware motor speed estimator", _njoints))
        {
            fprintf(stderr, "Using default value=5\n");
            for(i=1; i<_njoints+1; i++)
                _estim_params[i-1].mot_Vel_estimator_shift = 0;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _estim_params[i-1].mot_Vel_estimator_shift = xtmp.get(i).asInt();
        }

        /////// Joint Acceleration Estimation
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "JNT_accel_estimation", "a list of shift factors used by the firmware joint speed estimator", _njoints))
        {
            fprintf(stderr, "Using default value=5\n");
            for(i=1; i<_njoints+1; i++)
                _estim_params[i-1].jnt_Acc_estimator_shift = 0;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _estim_params[i-1].jnt_Acc_estimator_shift = xtmp.get(i).asInt();
        }

        /////// Motor Acceleration Estimation
        xtmp.clear();
        if (!extractGroup(velocityGroup, xtmp, "MOT_accel_estimation", "a list of shift factors used by the firmware motor speed estimator", _njoints))
        {
            fprintf(stderr, "Using default value=5\n");
            for(i=1; i<_njoints+1; i++)
                _estim_params[i-1].mot_Acc_estimator_shift = 5;   //Default value
        }
        else
        {
            for(i=1; i<xtmp.size(); i++)
                _estim_params[i-1].mot_Acc_estimator_shift = xtmp.get(i).asInt();
        }

    }
    else
    {
        fprintf(stderr, "A suitable value for [VELOCITY] Shifts was not found. Using default Shifts=4\n");
        for(i=1; i<_njoints+1; i++)
            _velocityShifts[i-1] = 0;   //Default value // not used now!! In the future this value may (should?) be read from config file and sertnto the EMS

        fprintf(stderr, "A suitable value for [VELOCITY] Timeout was not found. Using default Timeout=1000, i.e 1s.\n");
        for(i=1; i<_njoints+1; i++)
            _velocityTimeout[i-1] = 1000;   //Default value

        fprintf(stderr, "A suitable value for [VELOCITY] speed estimation was not found. Using default shift factor=5.\n");
        for(i=1; i<_njoints+1; i++)
        {
            _estim_params[i-1].jnt_Vel_estimator_shift = 0;   //Default value
            _estim_params[i-1].jnt_Acc_estimator_shift = 0;
            _estim_params[i-1].mot_Vel_estimator_shift = 0;
            _estim_params[i-1].mot_Acc_estimator_shift = 0;
        }
    }
    return true;
}

bool embObjMotionControl::init()
{
    eOmn_ropsigcfg_command_t 	*ropsigcfgassign;
    EOarray                     *array;
    eOropSIGcfg_t               sigcfg;
    int                         old = 0;
    eOprotID32_t                protid;

#ifdef _SETPOINT_TEST_
    eoy_sys_Initialise(NULL, NULL, NULL);
    int j, i;
    //init mutex
    for(j=0, i =0; j<  _njoints; j++, i++)
    {
        j_debug_data[i].mutex = Semaphore(1);
    }
#endif
    eOprotID32_t protid_ropsigcfgassign = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_ropsigcfg);


    EOnv *nvRoot_ropsigcfgassign;
    EOnv nv_ropsigcfgassign;
    nvRoot_ropsigcfgassign = res->getNVhandler(protid_ropsigcfgassign, &nv_ropsigcfgassign);


    ropsigcfgassign = (eOmn_ropsigcfg_command_t*) nvRoot_ropsigcfgassign->ram;
    array = (EOarray*) &ropsigcfgassign->array;
    eo_array_Reset(array);
    array->head.capacity = NUMOFROPSIGCFG;
    array->head.itemsize = sizeof(eOropSIGcfg_t);
    ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;

    ///////////////////////////////////////////////////
    // Configura le variabili da segnalare ogni ms   //
    ///////////////////////////////////////////////////

    int jStatusSize = sizeof(eOmc_joint_status_t);
    int mStatusSize = sizeof(eOmc_motor_status_basic_t);
    int totSigSize	= 0;

    for(int j=0; j< _njoints; j++)
    {
        //  yDebug() << "configuring ropSig for joint " << j;

        // Verify that the EMS is able to handle all those data. The macro EOK_HOSTTRANSCEIVER_capacityofropframeregulars has to be the one used by the firmware!!!!
        if( ! (EMS_capacityofropframeregulars >= (totSigSize += jStatusSize)) )
        {
            yError () << "No space left on EMS device for setting new regular messages!! Skipping remaining" << _fId.name;
            break;
        }

        // basterebbero jstatus__basic e jstatus__ofpid, ma la differenza tra questi due e il jstatus completo sono 4 byte, per ora non utilizzati.
       protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status);
        //    printf("\njointNVindex_jstatus nvid = %d (0x%04X)", nvid, nvid);
        if(EOK_uint16dummy == protid)
        {
            yError () << " NVID jointNVindex_jstatus not found for board " << _fId.boardNum  << " joint " << j;
        }
        else
        {
            sigcfg.id32 = protid;
            if(eores_OK != eo_array_PushBack(array, &sigcfg))
                yError () << " while loading ropSig Array for joint " << j << " at line " << __LINE__;
        }

        // Verify that the EMS is able to handle all those data. The macro EOK_HOSTTRANSCEIVER_capacityofropframeregulars has to be the one used by the firmware!!!!
        if( ! (EMS_capacityofropframeregulars >= (totSigSize += mStatusSize)) )
        {
            yError () << "No space left on EMS device for setting new regular messages!! Skipping remaining on board" << _fId.name;
            break;
        }

        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_status);
        //    printf("\nmotorNVindex_jstatus nvid = %d (0x%04X)", nvid, nvid);
        if(EOK_uint16dummy == protid)
        {
            yError () << " NVID eoprot_tag_mc_motor_status not found for board " << _fId.boardNum << " joint " << j;
        }
        else
        {
            sigcfg.id32 = protid;
            if(eores_OK != eo_array_PushBack(array, &sigcfg))
                yError () << " while loading ropSig Array for joint " << j << " at line " << __LINE__;
        }

        if( (NUMOFROPSIGCFG - 1) <= ((j - old +1)*2))	// a ropSigCfg can store only 20 variables at time. Send 2 messages if more are needed.
        {
            // A ropsigcfg vector can hold at max NUMOFROPSIGCFG (20) value. If more are needed, send another package,
            // so wait some time to let ethManager send this package and then start again.
            // yDebug() << "Maximun number of variables reached in the ropSigCfg array, splitting it in two pieces";
            if(!res->addSetMessage(protid_ropsigcfgassign, (uint8_t *) array))
            {
                yError() << "while setting rop sig cfg";
            }

            Time::delay(0.01);        // Wait here, the ethManager thread will take care of sending the loaded message
            eo_array_Reset(array);
            array->head.capacity = NUMOFROPSIGCFG;
            array->head.itemsize = sizeof(eOropSIGcfg_t);
            ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;
            old = j;
        }
 
    }

    // Send remaining stuff
    if( !res->addSetMessage(protid_ropsigcfgassign, (uint8_t *) array) )
    {
        yError() << "while setting rop sig cfg";
    }

    //////////////////////////////////////////
    // invia la configurazione dei GIUNTI   //
    //////////////////////////////////////////

    int jConfigSize 	= sizeof(eOmc_joint_config_t);
    int mConfigSize 	= sizeof(eOmc_motor_config_t);
    int totConfigSize	= 0;
    int index 				= 0;

//    EOnv *nvRoot;
//    EOnv nvtmp;


    // 	 yDebug() << "Sending joint configuration";
    if( EOK_HOSTTRANSCEIVER_capacityofrop < jConfigSize )
    {
        yError () << "Size of Joint Config is bigger than single ROP... cannot send it at all!! Fix it!!";
    }
    else
    {
        for(int logico=0; logico< _njoints; logico++)
        {
        	int fisico = _axisMap[logico];
            if( ! (EOK_HOSTTRANSCEIVER_capacityofropframeoccasionals >= (totConfigSize += jConfigSize)) )
            {
                // yDebug() << "Too many stuff to be sent at once... splitting in more messages";
                Time::delay(0.01);
                totConfigSize = 0;
            }
            protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, fisico, eoprot_tag_mc_joint_config);

//             if(EOK_uint16dummy == nvid)
//             {
//                 yError () << " NVID not found\n";
//                 continue;
//             }
//             nvRoot = res->getNVhandler((uint16_t)_fId.ep, nvid, &nvtmp);
// 
//             if(NULL == nvRoot)
//             {
//                 yError () << " NV pointer not found\n" << _fId.name << "board number " <<_fId.boardNum << "joint " << fisico << "at line" << __LINE__;
//                 continue;
//             }

            eOmc_joint_config_t     jconfig;
            memset(&jconfig, 0x00, sizeof(eOmc_joint_config_t));
            copyPid_iCub2eo(&_pids[logico],  &jconfig.pidposition);
            copyPid_iCub2eo(&_pids[logico],  &jconfig.pidvelocity);
            copyPid_iCub2eo(&_tpids[logico], &jconfig.pidtorque);

            jconfig.impedance.damping	= (eOmeas_damping_t) _impedance_params[logico].damping * 1000;
            jconfig.impedance.stiffness	= (eOmeas_stiffness_t) _impedance_params[logico].stiffness * 1000;
            jconfig.impedance.offset	= 0; //impedance_params[j];

            _cacheImpedance[logico].stiffness = jconfig.impedance.damping;
        	_cacheImpedance[logico].damping   = jconfig.impedance.stiffness;
        	_cacheImpedance[logico].offset = 0;

            jconfig.limitsofjoint.max = (eOmeas_position_t) convertA2I(_limitsMax[logico], _zeros[logico], _angleToEncoder[logico]);
            jconfig.limitsofjoint.min = (eOmeas_position_t) convertA2I(_limitsMin[logico], _zeros[logico], _angleToEncoder[logico]);
            jconfig.velocitysetpointtimeout = (eOmeas_time_t)_velocityTimeout[logico];
            jconfig.motionmonitormode = eomc_motionmonitormode_dontmonitor;

            jconfig.encoderconversionfactor = eo_common_float_to_Q17_14(_encoderconversionfactor[logico]);
            jconfig.encoderconversionoffset = eo_common_float_to_Q17_14(_encoderconversionoffset[logico]);

            if(!res->addSetMessage(protid, (uint8_t *) &jconfig))
            {
                yError() << "while setting joint config";
            }

            // Debugging... to much information can overload can queue on EMS
            Time::delay(0.01);
        }
    }

    //////////////////////////////////////////
    // invia la configurazione dei MOTORI   //
    //////////////////////////////////////////

//    yDebug() << "Sending motor MAX CURRENT ONLY";
    totConfigSize = 0;
    if( EOK_HOSTTRANSCEIVER_capacityofrop < mConfigSize )
    {
        yError () << "Size of Motor Config is bigger than single ROP... cannot send it at all!! Fix it";
    }
    else
    {
        for(int logico=0; logico< _njoints; logico++)
        {
        	int fisico = _axisMap[logico];
            if( ! (EOK_HOSTTRANSCEIVER_capacityofropframeoccasionals >= (totConfigSize += mConfigSize)) )
            {
                // 		yDebug() << "Too many stuff to be sent at once... splitting in more messages";
                Time::delay(0.01);
                totConfigSize = 0;
            }
            protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, fisico, eoprot_tag_mc_motor_config_maxcurrentofmotor);
            if(EOK_uint16dummy == protid)
            {
                yError () << " NVID not found\n";
                continue;
            }

            eOmeas_current_t	current = (eOmeas_current_t) _currentLimits[logico];

            if(!res->addSetMessage(protid, (uint8_t *) &current))
            {
                yError() << "while setting motor config";
            }
            // Debugging... to much information can overload can queue on EMS
            Time::delay(0.01);
        }
    }
    yTrace() << "EmbObj Motion Control for board " << _fId.boardNum << " istantiated correctly\n";
    return true;
}


bool embObjMotionControl::configure_mais(void)
{
    // Mais per lettura posizioni dita, c'e' solo sulle mani per ora


    uint8_t               maisnum   = 0;
    uint8_t               datarate  = 10;    //10 milli (like in icub_right_arm_safe.ini)  // type ok


    if((_fId.boardNum != 2) && (_fId.boardNum != 4))
    {
        return true;//only board 2 and 4 have mais connected
    }
    //set mais datarate = 1millisec
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_datarate);

    if(EOK_uint16dummy == protid)
    {
        yError () << "[eomc] NVID not found( maisNVindex_mconfig__datarate, " << _fId.name << "board number " << _fId.boardNum << "at line" << __LINE__ << ")";
        return false;
    }

    if(!res->addSetMessage(protid, &datarate))
    {
        yError() << "while setting mais datarate";
    }

    //set tx mode continuosly
    eOas_maismode_t     maismode  = eoas_maismode_txdatacontinuously;
    protid = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_mais, 0, eoprot_tag_as_mais_config_mode);
    if(EOK_uint16dummy == protid)
    {
        yError () << "[eomc] NVID not found( maisNVindex_mconfig__mode, " << _fId.name << "board number " << _fId.boardNum << "at line" << __LINE__ << ")";
        return false;
    }

    if(!res->addSetMessage(protid, (uint8_t *) &maismode))
    {
        yError() << "while setting mais maismode";
    }

    return true;
}

bool embObjMotionControl::close()
{
    ImplementControlMode::uninitialize();
    ImplementEncodersTimed::uninitialize();
    ImplementPositionControl2::uninitialize();
    ImplementVelocityControl<embObjMotionControl, IVelocityControl>::uninitialize();
    ImplementVelocityControl2::uninitialize();
    ImplementPidControl<embObjMotionControl, IPidControl>::uninitialize();
    ImplementControlCalibration2<embObjMotionControl, IControlCalibration2>::uninitialize();
    ImplementAmplifierControl<embObjMotionControl, IAmplifierControl>::uninitialize();
    ImplementImpedanceControl::uninitialize();
    ImplementControlLimits2::uninitialize();
    ImplementPositionDirect::uninitialize();
    int ret = ethManager->releaseResource(_fId);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
    return true;
}

FEAT_ID embObjMotionControl::getFeat_id()
{
    return(this->_fId);
}

// TODO da eliminare?
eoThreadEntry * embObjMotionControl::appendWaitRequest(int j, uint32_t protoid)
{
    eoRequest req;
    if(!requestQueue->threadPool->getId(&req.threadId) )
        yError() << "Error: too much threads!! (embObjMotionControl)";
    req.joint = j;
    req.nvid = res->translate_NVid2index(_fId.boardNum, protoid);
    requestQueue->append(req);
    return requestQueue->threadPool->getThreadTable(req.threadId);
}

void embObjMotionControl::refreshEncoderTimeStamp(int joint)
{
    static long int count = 0;
    count++;

    // for this initted flag only one 
    if(initted)
    {
        _mutex.wait();
        _encodersStamp[joint] = Time::now();
        _mutex.post();
    }
}

///////////// PID INTERFACE

bool embObjMotionControl::setPidRaw(int j, const Pid &pid)
{
//    EOnv tmp;
    eOprotID32_t protoId = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidposition);
    eOmc_PID_t  outPid;
    copyPid_iCub2eo(&pid, &outPid);

    if(!res->addSetMessage(protoId, (uint8_t *) &outPid))
    {
        yError() << "while setting position PIDs for board" << _fId.boardNum << "joint " << j;
        return false;
    }

    // Now set the velocity pid too...
    protoId = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidvelocity);
    if(!res->addSetMessage(protoId, (uint8_t *) &outPid))
    {
        yError() << "while setting velocity PIDs for board" << _fId.boardNum << "joint " << j;
        return false;
    }
    return true;
}

bool embObjMotionControl::setPidsRaw(const Pid *pids)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= setPidRaw(j, pids[j]);
    }
    return ret;
}

bool embObjMotionControl::setReferenceRaw(int j, double ref)
{
    eOprotID32_t protoId = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    eOmc_setpoint_t setpoint;
    uint16_t *prog = (uint16_t*) &setpoint;

    setpoint.type = (eOenum08_t) eomc_setpoint_positionraw;
    setpoint.to.position.value = (eOmeas_position_t) ref;
    setpoint.to.position.withvelocity = 0;
#ifdef VERIFY_ROP_SETPOSITIONRAW
    refRawSignature[j]++;

    switch(_fId.boardNum)
    {
        case 1:
        case 2:
        case 3:
        case 4:
            if( (!sendingDirects) && (j<5) )
            {
                yWarning() << "Sending positionDirect/setReference for board" << _fId.boardNum << "joint " << j;
                sendingDirects[j]=true;
            }
    }

    return res->addSetMessageWithSignature(nvid, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (uint8_t*) &setpoint, refRawSignature[j]);
#endif 

    return res->addSetMessage(protoId, (uint8_t*) &setpoint);
}

bool embObjMotionControl::setReferencesRaw(const double *refs)
{
    bool ret = true;
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        ret &= setReferenceRaw(j, refs[index]);
    }
    return ret;
}

bool embObjMotionControl::setErrorLimitRaw(int j, double limit)
{
    // print_debug(AC_trace_file, "embObjMotionControl::setErrorLimitRaw()");
    return NOT_YET_IMPLEMENTED("setErrorLimitRaw");
}

bool embObjMotionControl::setErrorLimitsRaw(const double *limits)
{
    // print_debug(AC_trace_file, "embObjMotionControl::setErrorLimitsRaw()");
    return NOT_YET_IMPLEMENTED("setErrorLimitsRaw");
}

bool embObjMotionControl::getErrorRaw(int j, double *err)
{
//    EOnv tmp;
    int mycontrolMode;
    /* Values in pid.XXX fields are valid ONLY IF we are in the corresponding control mode.
    Read it from the signalled message so we are sure that mode and pid values are coherent to each other */
    getControlModeRaw(j, &mycontrolMode);
    if(VOCAB_CM_POSITION != mycontrolMode )
    {
        //yWarning() << "Asked for Position PID Error while not in Position control mode. Returning zeros";
        err = 0;
        return false;
    }

    eOprotID32_t protoId = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);
    uint16_t size;
    eOmc_joint_status_ofpid_t  tmpJointStatus;
    res->readBufferedValue(protoId, (uint8_t *)&tmpJointStatus, &size);
    *err = (double) tmpJointStatus.error;
    return true;
}

bool embObjMotionControl::getErrorsRaw(double *errs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getErrorRaw(j, &errs[j]);
    }
    return ret;
}

bool embObjMotionControl::getOutputRaw(int j, double *out)
{
//    EOnv tmp;
    bool ret = true;
    eOprotID32_t protoId = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);
    uint16_t size;
    eOmc_joint_status_ofpid_t  tmpJointStatus;
    if(res->readBufferedValue(protoId, (uint8_t *)&tmpJointStatus, &size) )
    {
        *out = (double) tmpJointStatus.output;
    }
    else
    {
        *out = 0;
        ret = false;
    }
    return ret;
}

bool embObjMotionControl::getOutputsRaw(double *outs)
{
    bool ret = true;

    for(int j=0; j< _njoints; j++)
    {
        ret &= getOutputRaw(j, &outs[j]);
    }
    return ret;
}

bool embObjMotionControl::getPidRaw(int j, Pid *pid)
{
    eOprotID32_t protid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidposition);
    // Sign up for waiting the reply

    eoThreadEntry *tt = appendWaitRequest(j, protid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if(!res->addGetMessage(protid) )
    {
        yError() << "Can't send get pid request for board" << _fId.boardNum << "joint " << j;
        return false;
    }

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "getPid timed out for board"<< _fId.boardNum << " joint " << j;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_PID_t eoPID;
    res->readBufferedValue(protid, (uint8_t *)&eoPID, &size);

    copyPid_eo2iCub(&eoPID, pid);

    return true;
}

bool embObjMotionControl::getPidsRaw(Pid *pids)
{
    bool ret = true;

    // just one joint at time, wait answer before getting to the next.
    // This is because otherwise too many msg will be placed into can queue
    for(int j=0, index=0; j<_njoints; j++, index++)
    {
        ret &=getPidRaw(j, &pids[j]);
    }
    return ret;
}

bool embObjMotionControl::getReferenceRaw(int j, double *ref)
{
    eOprotID32_t protoId = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);
    uint16_t size;
    eOmc_joint_status_ofpid_t  tmpJointStatus;
    res->readBufferedValue(protoId, (uint8_t *)&tmpJointStatus, &size);
    *ref = tmpJointStatus.reference;
    return true;
}

bool embObjMotionControl::getReferencesRaw(double *refs)
{
    bool ret = true;

    // just one joint at time, wait answer before getting to the next.
    // This is because otherwise too many msg will be placed into can queue
    for(int j=0; j< _njoints; j++)
    {
        ret &= getReferenceRaw(j, &refs[j]);
    }
    return ret;
}

bool embObjMotionControl::getErrorLimitRaw(int j, double *limit)
{
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool embObjMotionControl::getErrorLimitsRaw(double *limits)
{
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool embObjMotionControl::resetPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetPid");
}

bool embObjMotionControl::disablePidRaw(int j)
{
//    EOnv tmp;
    // Spegni tutto!! Setta anche Amp off
     eOprotID32_t protid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);
    _enabledAmp[j ] = false;
    _enabledPid[j ] = false;

    eOmc_controlmode_command_t val = eomc_controlmode_cmd_switch_everything_off;
    if(! res->addSetMessage(protid, (uint8_t *) &val) )
    {
        yError() << "while disabling pid";
        return false;
    }
    return true;
}

bool embObjMotionControl::enablePidRaw(int j)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);

    // se giunto non è calibrato non fa nulla, se è calibrato manda il control mode position
    _enabledPid[j ] = true;


    if(_calibrated[j ])
    {
        eOmc_controlmode_command_t val = eomc_controlmode_cmd_position;

        if(! res->addSetMessage(protid, (uint8_t *) &val))
        {
            yError() << "while enabling pid";
            return false;
        }
    }
    return true;
}

bool embObjMotionControl::setOffsetRaw(int j, double v)
{
    return NOT_YET_IMPLEMENTED("setOffset");
}

////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool embObjMotionControl::setVelocityModeRaw()
{
    bool ret = true;

    eOprotID32_t protid;

    eOmc_controlmode_command_t val = eomc_controlmode_cmd_velocity;
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);

        if(! res->addSetMessage(protid, (uint8_t *) &val))
        {
            yError() << "while setting velocity mode";
            return false;
        }
    }
    return ret;
}

bool embObjMotionControl::velocityMoveRaw(int j, double sp)
{
    int index = j ;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);

    _command_speeds[j] = sp ;   // save internally the new value of speed.

    eOmc_setpoint_t setpoint;
    setpoint.type = eomc_setpoint_velocity;
    setpoint.to.velocity.value =  (eOmeas_velocity_t) _command_speeds[index];
    setpoint.to.velocity.withacceleration = (eOmeas_acceleration_t) _ref_accs[index];


    if(! res->addSetMessage(protid, (uint8_t *) &setpoint))
    {
        yError() << "while setting velocity mode";
        return false;
    }
    return true;
}

bool embObjMotionControl::velocityMoveRaw(const double *sp)
{
    bool ret = true;
    eOmc_setpoint_t setpoint;

    setpoint.type = eomc_setpoint_velocity;

    for(int j=0; j< _njoints; j++)
    {
        ret = velocityMoveRaw(j, sp[j]) && ret;
    }

    return ret;
}


////////////////////////////////////////
//    Calibration control interface   //
////////////////////////////////////////

bool embObjMotionControl::calibrate2Raw(int j, unsigned int type, double p1, double p2, double p3)
{
    yTrace() << "calibrate2Raw for BOARD " << _fId.boardNum << "joint" << j;

    // Tenere il check o forzare questi sottostati?
//    if(!_enabledAmp[j ] )
//    {
//        yWarning () << "Called calibrate for joint " << j << "with PWM(AMP) not enabled, forcing it!!";
//        //		return false;
//    }

//    if(!_enabledPid[j ])
//    {
//        yWarning () << "Called calibrate for joint " << j << "with PID not enabled, forcing it!!";
//        //		return false;
//    }

    //   There is no explicit command "go to calibration mode" but it is implicit in the calibration command

    // Get calibration command NV pointer
//    EOnv tmp_calib;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_calibration);
    eOmc_calibrator_t calib;
    memset(&calib, 0x00, sizeof(calib));
    calib.type = type;

    switch(type)
    {
        // muove -> amp+pid, poi calib
    case eomc_calibration_type0_hard_stops:
        calib.params.type0.pwmlimit = (int16_t) p1;
        calib.params.type0.velocity = (eOmeas_velocity_t) p2;
        break;

        // fermo
    case eomc_calibration_type1_abs_sens_analog:
        calib.params.type1.position = (int16_t) p1;
        calib.params.type1.velocity = (eOmeas_velocity_t) p2;
        break;

        // muove
    case eomc_calibration_type2_hard_stops_diff:
        calib.params.type2.pwmlimit = (int16_t) p1;
        calib.params.type2.velocity = (eOmeas_velocity_t) p2;
        break;

        // muove
    case eomc_calibration_type3_abs_sens_digital:
        calib.params.type3.position = (int16_t) p1;
        calib.params.type3.velocity = (eOmeas_velocity_t) p2;
        calib.params.type3.offset   = (int32_t) p3;
        break;

        // muove
    case eomc_calibration_type4_abs_and_incremental:
        calib.params.type4.position   = (int16_t) p1;
        calib.params.type4.velocity   = (eOmeas_velocity_t) p2;
        calib.params.type4.maxencoder = (int32_t) p3;
        break;

    default:
        yError () << "Calibration type unknown!! (embObjMotionControl)\n";
        return false;
        break;
    }

    if(! res->addSetMessage(protid, (uint8_t *) &calib))
    {
        yError() << "while setting velocity mode";
        return false;
    }

    _calibrated[j ] = true;

    return true;
}

bool embObjMotionControl::doneRaw(int axis)
{
    // used only in calibration procedure, for normal work use the checkMotionDone

    bool result = false;
    uint16_t size;
    eOmc_controlmode_t type;
    eOmc_joint_status_basic_t status;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_basic);
    //EO_WARNING("acemor-> see wait of 100 msec in embObjMotionControl::doneRaw(int axis)")
    // acemor on 21 may 2013: the delay is not necessary in normal cases when the joint cleanly starts in eomc_controlmode_idle and the calibrator sets it to eomc_controlmode_calib.
    // however we keep it because: ... sometimes the boards are not started cleanly (see later note1) and the controlmodestatus can contain an invalid value.
    //                                 the value becomes eomc_controlmode_calib only after the calibration command has arrived to the ems (and if relevant then to the can board and it status
    //                                 is signalled back). 
    // thus: al least for the first call after a calibration command on that joint we should keep the delay. Moreover: a delay of 1 sec is 
    // in parametricCalibrator::checkCalibrateJointEnded() just before calling doneRaw().

    Time::delay(0.1);   // EO_WARNING()

    res->readBufferedValue(protid, (uint8_t*) &status, &size);
    type = (eOmc_controlmode_t) status.controlmodestatus;

    // if the control mode is no longer a calibration type, it means calibration ended
    if( (eomc_controlmode_idle == type) || (eomc_controlmode_calib == type) )
        result = false;
    else
        result = true;
//    yWarning() << _fId.name << "joint " << axis << "Calibration done is " << result;
    return result;
}

////////////////////////////////////////
//     Position control interface     //
////////////////////////////////////////

bool embObjMotionControl::getAxes(int *ax)
{
    *ax=_njoints;

    return true;
}

bool embObjMotionControl::setPositionModeRaw()
{
    bool ret = true;

    eOmc_controlmode_command_t val = eomc_controlmode_cmd_position;
    for(int j=0; j< _njoints; j++)
    {
       if(!_calibrated[j])
       {
            yWarning() << "called position mode on a non calibrated axes... skipping";
            return false;
       }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);

        ret &= res->addSetMessage(protid, (uint8_t*) &val);
    }

    return ret;
}

bool embObjMotionControl::positionMoveRaw(int j, double ref)
{
#ifdef _SETPOINT_TEST_
    /*
    static eOnvID_t nvid_aux_6 = 0, nvid_aux_8=0;

    if((nvid_aux_6 == 0) && (_fId.ep == 22))
    {
        int j, i;
        for(j=0, i =0; j<  _njoints; j++, i++)
    	{
        	eOnvID_t nvid_aux_6 = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) i, jointNVindex_jcmmnds__setpoint );
        	yError() << "++++++++++++++++++++++++ board num 6"<<  "joint" << i << "ep = " << _fId.ep << "nvid = " << nvid_aux_6;
    	}

    }

    if((nvid_aux_8 == 0) && (_fId.ep == 24))
    {
        int j, i;
        for(j=0, i =0; j<  _njoints; j++, i++)
    	{
			eOnvID_t nvid_aux_8 = eo_cfg_nvsEP_mc_joint_NVID_Get((eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (eOcfg_nvsEP_mc_jointNumber_t) i, jointNVindex_jcmmnds__setpoint );
			yError() << "++++++++++++++++++++++++ board num 8"<<  "joint" << i << "ep = " << _fId.ep << "nvid = " << nvid_aux_8;
    	}
    }
    */
#endif

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);


    _ref_positions[j] = ref;   // save internally the new value of pos.

    eOmc_setpoint_t setpoint;
    uint16_t *prog = (uint16_t*) &setpoint;

    setpoint.type = (eOenum08_t) eomc_setpoint_position;
    setpoint.to.position.value =  (eOmeas_position_t) _ref_positions[j];
    setpoint.to.position.withvelocity = (eOmeas_velocity_t) _ref_speeds[j];
#ifdef _SETPOINT_TEST_
    if( (_fId.boardNum == 6) || (_fId.boardNum==8 ) )
    {
    	j_debug_data[j].mutex.wait();
    	if(!j_debug_data[j].gotIt)
    	{
    		struct  timeval 	err_time;
		    gettimeofday(&err_time, NULL);
    		yError() << "[" << err_time.tv_sec <<"." <<err_time.tv_usec << "] for EMS" << _fId.boardNum << "joint " << j << "Trying to send a new setpoint wihout ACK!!!!!!";
    	}

    	j_debug_data[j].gotIt = false;
    	j_debug_data[j].last_pos = j_debug_data[j].pos;
    	j_debug_data[j].pos = setpoint.to.position.value;
    	j_debug_data[j].wtf = false;
	    j_debug_data[j].count_old = 0;

    	j_debug_data[j].mutex.post();
    }
#endif

#ifdef VERIFY_ROP_SETPOSITIONRAW
    switch(_fId.boardNum)
    {
        case 1:
        case 2:
        case 3:
        case 4:
            if( (sendingDirects) && (j<5) )
            {
                yWarning() << "Sending positionMove for board" << _fId.boardNum << "joint " << j;
                sendingDirects[j]=false;
            }
    }
#endif

//    yDebug() << "Position move EP" << _fId.ep << "j" << j << setpoint.to.position.value << "\tspeed " << setpoint.to.position.withvelocity  << " at time: " << (Time::now()/1e6);

    return res->addSetMessage(protid, (uint8_t*) &setpoint);
}

bool embObjMotionControl::positionMoveRaw(const double *refs)
{
    bool ret = true;

    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        ret &= positionMoveRaw(j, refs[index]);
    }
    return ret;
}

bool embObjMotionControl::relativeMoveRaw(int j, double delta)
{
    return NOT_YET_IMPLEMENTED("positionRelative");
}

bool embObjMotionControl::relativeMoveRaw(const double *deltas)
{
    return NOT_YET_IMPLEMENTED("positionRelative");
}

bool embObjMotionControl::checkMotionDoneRaw(int j, bool *flag)
{
    EOnv tmpnv_config;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_motionmonitormode);

//    EOnv *nvRoot_config = res->getNVhandler(protid, &tmpnv_config);
//
//    //	printf("nvid of check motion done 0x%04X\n", nvid_config);
//    if(NULL == nvRoot_config)
//    {
//        NV_NOT_FOUND;
//        return false;
//    }

    // monitor status until set point is reached, if it wasn't already set
    // this is because the function has to be in a non blocking fashion and I want to avoid resending the same message over and over again!!

    if(!checking_motiondone[j ])
    {
        checking_motiondone[j ] = true;

        eOmc_motionmonitormode_t tmp = eomc_motionmonitormode_forever;

        res->addSetMessage(protid,(uint8_t *) &tmp);
//         if( !res->nvSetData(nvRoot_config, &tmp, eobool_true, eo_nv_upd_dontdo))
//         {
//             // print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
//             return false;
//         }
//         if(!res->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_config) )
//             return false;
    }


    // Read the current value - it is signalled spontaneously every cycle, so we don't have to wait here
//    EOnv tmpnv_status;
    protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);
//     EOnv *nvRoot_status = res->getNVhandler((uint16_t)_fId.ep, nvid_status, &tmpnv_status);

    uint16_t size;
    eOmc_joint_status_basic_t status;
//     res->getNVvalue(nvRoot_status, (uint8_t *) &status, &size);

    res->readBufferedValue(protid,(uint8_t *) &status, &size);
    if(eomc_motionmonitorstatus_setpointisreached == status.motionmonitorstatus)
    {
        *flag = true;

        // to stop monitoring when set point is reached... this create problems when using the other version of
        // the function with all axis togheter. A for loop cannot be used. Skip it for now
        //		eOmc_motionmonitormode_t tmp = eomc_motionmonitormode_dontmonitor;
        //		if( eomc_motionmonitormode_dontmonitor != *nvRoot_config->motionmonitormode)
        //		{
        //			if( !res->nvSetData(nvRoot_config, &val, eobool_true, eo_nv_upd_dontdo))
        //			{
        //				// print_debug(AC_error_file, "\n>>> ERROR eo_nv_Set !!\n");
        //				return false;
        //			}
        //			res->load_occasional_rop(eo_ropcode_set, (uint16_t)_fId.ep, nvid_config);
        //		}
        //		checking_motiondone[j ]= false;
    }
    else
        *flag = false;
    return true;
}

bool embObjMotionControl::checkMotionDoneRaw(bool *flag)
{
    bool ret = true;
    bool val, tot_res = true;

    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        ret &= checkMotionDoneRaw(&val);
        tot_res &= val;
    }
    *flag = tot_res;
    return ret;
}

bool embObjMotionControl::setRefSpeedRaw(int j, double sp)
{
    // Velocity is expressed in iDegrees/s
    // save internally the new value of speed; it'll be used in the positionMove
    int index = j ;
    _ref_speeds[index] = sp;
    return true;
}

bool embObjMotionControl::setRefSpeedsRaw(const double *spds)
{
    // Velocity is expressed in iDegrees/s
    // save internally the new value of speed; it'll be used in the positionMove
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        _ref_speeds[index] = spds[index];
    }
    return true;
}

bool embObjMotionControl::setRefAccelerationRaw(int j, double acc)
{
    // Acceleration is expressed in iDegrees/s^2
    // save internally the new value of the acceleration; it'll be used in the velocityMove command

    if (acc > 1e6)
    {
        _ref_accs[j ] =  1e6;
    }
    else if (acc < -1e6)
    {
        _ref_accs[j ] = -1e6;
    }
    else
    {
        _ref_accs[j ] = acc;
    }

    return true;
}

bool embObjMotionControl::setRefAccelerationsRaw(const double *accs)
{
    // Acceleration is expressed in iDegrees/s^2
    // save internally the new value of the acceleration; it'll be used in the velocityMove command
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        if (accs[j] > 1e6)
        {
            _ref_accs[index] =  1e6;
        }
        else if (accs[j] < -1e6)
        {
            _ref_accs[index] = -1e6;
        }
        else
        {
            _ref_accs[index] = accs[j];
        }
    }
    return true;
}

bool embObjMotionControl::getRefSpeedRaw(int j, double *spd)
{
    *spd = _ref_speeds[j];
    return true;
}

bool embObjMotionControl::getRefSpeedsRaw(double *spds)
{
    memcpy(spds, _ref_speeds, sizeof(double) * _njoints);
    return true;
}

bool embObjMotionControl::getRefAccelerationRaw(int j, double *acc)
{
    *acc = _ref_accs[j];
    return true;
}

bool embObjMotionControl::getRefAccelerationsRaw(double *accs)
{
    memcpy(accs, _ref_accs, sizeof(double) * _njoints);
    return true;
}

bool embObjMotionControl::stopRaw(int j)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_stoptrajectory);

    eObool_t stop = eobool_true;
    
#ifdef VERIFY_ROP_SETPOSITIONRAW
    switch(_fId.boardNum)
    {
        case 1:
        case 2:
        case 3:
        case 4:
            if(j<5)
            {
                yWarning() << "Sending stop for board" << _fId.boardNum << "joint " << j;
            }
    }
#endif
    return res->addSetMessage(protid, (uint8_t*) &stop);
}

bool embObjMotionControl::stopRaw()
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= stopRaw(j);
    }
    return ret;
}
///////////// END Position Control INTERFACE  //////////////////

////////////////////////////////////////
//     Position control2 interface    //
////////////////////////////////////////

bool embObjMotionControl::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&positionMoveRaw(joints[j], refs[j]);
    }
    return ret;
}

bool embObjMotionControl::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&relativeMoveRaw(joints[j], deltas[j]);
    }
    return ret;
}

bool embObjMotionControl::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flag)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && checkMotionDoneRaw(joints[j], flag);
    }
    return ret;
}

bool embObjMotionControl::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&setRefSpeedRaw(joints[j], spds[j]);
    }
    return ret;
}

bool embObjMotionControl::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&setRefAccelerationRaw(joints[j], accs[j]);
    }
    return ret;
}

bool embObjMotionControl::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && getRefSpeedRaw(joints[j], &spds[j]);
    }
    return ret;
}

bool embObjMotionControl::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && getRefAccelerationRaw(joints[j], &accs[j]);
    }
    return ret;
}

bool embObjMotionControl::stopRaw(const int n_joint, const int *joints)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&stopRaw(joints[j]);
    }
    return ret;
}

///////////// END Position Control INTERFACE  //////////////////

// ControlMode
bool embObjMotionControl::setPositionModeRaw(int j)
{
    eOmc_controlmode_command_t  val = eomc_controlmode_cmd_position;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);

    return res->addSetMessage(protid, (uint8_t*) &val);
}

bool embObjMotionControl::setVelocityModeRaw(int j)
{
    eOmc_controlmode_command_t    val = eomc_controlmode_cmd_velocity;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);

    return res->addSetMessage(protid, (uint8_t*) &val);

}

bool embObjMotionControl::setTorqueModeRaw(int j)
{

    eOmc_controlmode_command_t    val =  eomc_controlmode_cmd_torque;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);

    return res->addSetMessage(protid, (uint8_t*) &val);

}

bool embObjMotionControl::setImpedancePositionModeRaw(int j)
{
    eOmc_controlmode_command_t    val = eomc_controlmode_cmd_impedance_pos;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);

    return res->addSetMessage(protid, (uint8_t*) &val);
}

bool embObjMotionControl::setImpedanceVelocityModeRaw(int j)
{
    eOmc_controlmode_command_t    val = eomc_controlmode_cmd_impedance_vel;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);
    return res->addSetMessage(protid, (uint8_t*) &val);
}

bool embObjMotionControl::setOpenLoopModeRaw(int j)
{
    eOmc_controlmode_command_t    val = eomc_controlmode_cmd_openloop;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);
    return res->addSetMessage(protid, (uint8_t*) &val);
}

bool embObjMotionControl::getControlModeRaw(int j, int *v)
{
    uint16_t                      size;
    eOmc_joint_status_basic_t     status;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);
    bool ret = res->readBufferedValue(protid, (uint8_t *)&status, &size);

    eOmc_controlmode_t type = (eOmc_controlmode_t) status.controlmodestatus;

    switch(type)
    {
    case eomc_controlmode_idle:
        *v=VOCAB_CM_IDLE;
        //	        printf("IDLE\n");
        break;

    case eomc_controlmode_position:
        *v=VOCAB_CM_POSITION;
        //	        printf("POSITION\n");
        break;

    case eomc_controlmode_velocity:
        *v=VOCAB_CM_VELOCITY;
        //	        printf("VELOCITY\n");
        break;

    case eomc_controlmode_torque:
        *v=VOCAB_CM_TORQUE;
        //	        printf("TORQUE\n");
        break;

    case eomc_controlmode_calib:
        *v=VOCAB_CM_UNKNOWN;
        //			printf("CALIBRATING\n");
        break;

    case eomc_controlmode_impedance_pos:
        *v=VOCAB_CM_IMPEDANCE_POS;
        //	        printf("IMPEDANCE_POS\n");
        break;

    case eomc_controlmode_impedance_vel:
        *v=VOCAB_CM_IMPEDANCE_VEL;
        //	        printf("IMPEDANCE_VEL\n");
        break;

    case eomc_controlmode_openloop:
        *v=VOCAB_CM_OPENLOOP;
        //	        printf("VOCAB_CM_OPENLOOP\n");
        break;

    default:
        *v=VOCAB_CM_UNKNOWN;
        //	        printf("UNKNOWN (0x%04X)\n", status.controlmodestatus);
        break;
    }
    return true;
}

bool embObjMotionControl::getControlModesRaw(int* v)
{
    bool ret = true;
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        ret &= getControlModeRaw(j, &v[index]);
    }
    return ret;
}

//////////////////////// BEGIN EncoderInterface

bool embObjMotionControl::setEncoderRaw(int j, double val)
{
    return NOT_YET_IMPLEMENTED("setEncoder");
}

bool embObjMotionControl::setEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setEncoders");
}

bool embObjMotionControl::resetEncoderRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetEncoder");
}

bool embObjMotionControl::resetEncodersRaw()
{
    return NOT_YET_IMPLEMENTED("resetEncoders");
}

bool embObjMotionControl::getEncoderRaw(int j, double *value)
{
    uint16_t      size;
    eOmc_joint_status_basic_t     status;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);

    bool ret = res->readBufferedValue(protid, (uint8_t *)&status, &size);

    if(ret)
    {
        eOmc_controlmode_t type = (eOmc_controlmode_t) status.controlmodestatus;
        *value = (double) status.position;
    }
    else
    {
        yError() << "embObjMotionControl while reading encoder";
        *value = 0;
    }

    return ret;
}

bool embObjMotionControl::getEncodersRaw(double *encs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getEncoderRaw(j, &encs[j]);

    }
    return ret;
}

bool embObjMotionControl::getEncoderSpeedRaw(int j, double *sp)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);
    uint16_t      size;
    eOmc_joint_status_basic_t  tmpJointStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpJointStatus, &size);
    // extract requested data from status
    *sp = (double) tmpJointStatus.velocity;
    return true;
}

bool embObjMotionControl::getEncoderSpeedsRaw(double *spds)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getEncoderSpeedRaw(j, &spds[j]);
    }
    return ret;
}

bool embObjMotionControl::getEncoderAccelerationRaw(int j, double *acc)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_basic);
    uint16_t      size;
    eOmc_joint_status_basic_t  tmpJointStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpJointStatus, &size);
    *acc = (double) tmpJointStatus.acceleration;
    return true;
}

bool embObjMotionControl::getEncoderAccelerationsRaw(double *accs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getEncoderAccelerationRaw(j, &accs[j]);
    }
    return ret;
}

///////////////////////// END Encoder Interface

bool embObjMotionControl::getEncodersTimedRaw(double *encs, double *stamps)
{
    bool ret = getEncodersRaw(encs);
    _mutex.wait();
    for(int i=0; i<_njoints; i++)
        stamps[i] = _encodersStamp[i];

    double tmp =  _encodersStamp[0];
    _mutex.post();

    return ret;
}

bool embObjMotionControl::getEncoderTimedRaw(int j, double *encs, double *stamp)
{
    bool ret = getEncoderRaw(j, encs);
    _mutex.wait();
    *stamp = _encodersStamp[j];
    _mutex.post();

    return ret;
}

////// Amplifier interface

bool embObjMotionControl::enableAmpRaw(int j)
{
    // Just take note of this command. Does nothing here... wait for enable pid
    _enabledAmp[j ] = true;
    return true;
}

bool embObjMotionControl::disableAmpRaw(int j)
{
    // Spegni tutto!! Setta anche pid off
    _enabledAmp[j ] = false;      // da proteggere anche questa scrittura interna??
    _enabledPid[j ] = false;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);
    // yDebug() << "disableAmpRaw AMP status " << _enabledAmp[j ];
    eOmc_controlmode_command_t val = eomc_controlmode_cmd_switch_everything_off;
    return res->addSetMessage(protid, (uint8_t*) &val);
}

bool embObjMotionControl::getCurrentRaw(int j, double *value)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_status_basic);
    uint16_t size;
    eOmc_motor_status_basic_t  tmpMotorStatus;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&tmpMotorStatus, &size);

    *value = (double) tmpMotorStatus.current;
    return true;
}

bool embObjMotionControl::getCurrentsRaw(double *vals)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getCurrentRaw(j, &vals[j]);
    }
    return ret;
}

bool embObjMotionControl::setMaxCurrentRaw(int j, double val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_maxcurrentofmotor);
    eOmeas_current_t  maxCurrent = (eOmeas_current_t) val;
    return res->addSetMessage(protid, (uint8_t*) &val);
}

bool embObjMotionControl::getAmpStatusRaw(int j, int *st)
{
    (_enabledAmp[j ]) ? *st = 1 : *st = 0;
    return true;
}

bool embObjMotionControl::getAmpStatusRaw(int *sts)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
    {
        sts[j] = _enabledAmp[j];
    }

    return ret;
}

#ifdef IMPLEMENT_DEBUG_INTERFACE
//----------------------------------------------\\
//	Debug interface
//----------------------------------------------\\

bool embObjMotionControl::setParameterRaw(int j, unsigned int type, double value)   { }
bool embObjMotionControl::getParameterRaw(int j, unsigned int type, double* value)  { }
bool embObjMotionControl::getDebugParameterRaw(int j, unsigned int index, double* value)  { }
bool embObjMotionControl::setDebugParameterRaw(int j, unsigned int index, double value)   { }
bool embObjMotionControl::setDebugReferencePositionRaw(int j, double value)         { }
bool embObjMotionControl::getDebugReferencePositionRaw(int j, double* value)        { }
bool embObjMotionControl::getRotorPositionRaw         (int j, double* value)        { }
bool embObjMotionControl::getRotorPositionsRaw        (double* value)               { }
bool embObjMotionControl::getRotorSpeedRaw            (int j, double* value)        { }
bool embObjMotionControl::getRotorSpeedsRaw           (double* value)               { }
bool embObjMotionControl::getRotorAccelerationRaw     (int j, double* value)        { }
bool embObjMotionControl::getRotorAccelerationsRaw    (double* value)               { }
bool embObjMotionControl::getJointPositionRaw         (int j, double* value)        { }
bool embObjMotionControl::getJointPositionsRaw        (double* value)               { }
#endif

// Limit interface
bool embObjMotionControl::setLimitsRaw(int j, double min, double max)
{
    bool ret = true;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_limitsofjoint);


    eOmeas_position_limits_t    limits;
    limits.max = (eOmeas_position_t) convertA2I(max, _zeros[j], _angleToEncoder[j]);
    limits.min = (eOmeas_position_t) convertA2I(min, _zeros[j], _angleToEncoder[j]);

    ret = res->addSetMessage(protid, (uint8_t *) &limits);


    if(!ret)
    {
        yError() << "while setting position limits for joint" << j << " \n";
    }
    return ret;
}

bool embObjMotionControl::getLimitsRaw(int j, double *min, double *max)
{
    eOmeas_position_limits_t limits;
    eOprotID32_t protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_limitsofjoint);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if(!res->addGetMessage(protoid) )
    {
        yError() << "Can't send get min position limit request for board" << _fId.boardNum << "joint " << j;
        return false;
    }

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        printf("\n\n-----------------------\nTIMEOUT for joint %d\n-----------------------\n", j); //yError () << "ask request timed out, joint " << j;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }
    // Get the value
    uint16_t size;

    bool ret = res->readBufferedValue(protoid, (uint8_t *)&limits, &size);

    *min = (double)limits.min + SAFETY_THRESHOLD;
    *max = (double)limits.max - SAFETY_THRESHOLD;
    return ret;
}

// IControlLimits2
bool embObjMotionControl::setVelLimitsRaw(int axis, double min, double max)
{
    return NOT_YET_IMPLEMENTED("setVelLimitsRaw");
}

bool embObjMotionControl::getVelLimitsRaw(int axis, double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getVelLimitsRaw");
}


/*
 * IVirtualAnalogSensor Interface
 *
 *  DEPRECATED!! WILL BE REMOVED IN THE NEAR FUTURE!!
 *
 */

int embObjMotionControl::getState(int ch)
{
    return VAS_OK;
};

int embObjMotionControl::getChannels()
{
    return _njoints;
};

bool embObjMotionControl::updateMeasure(yarp::sig::Vector &fTorques)
{
    bool ret = true;

    for(int j=0; j< _njoints; j++)
    {
        ret = ret && updateMeasure(j, fTorques[j]);
    }
    return ret;
}

bool embObjMotionControl::updateMeasure(int j, double &fTorque)
{

	double NEWTON2SCALE=32767.0/_maxTorque[j];

	eOmeas_torque_t meas_torque = 0;

	if(0 != _maxTorque[j])
	{
	//    EOnv tmp;
	//    if (fTorque < -1000.0 || fTorque > 1000.0) fTorque = 0.0;

		if(fTorque < (- _maxTorque[j] ))
		{
			fTorque = (- _maxTorque[j]);
		}
		if(fTorque > _maxTorque[j])
		{
			fTorque = _maxTorque[j];
		}

		meas_torque = (eOmeas_torque_t)(NEWTON2SCALE*fTorque);
	}

    eOprotID32_t protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_inputs_externallymeasuredtorque);
    return res->addSetMessage(protoid, (uint8_t*) &meas_torque);
}

// end  IVirtualAnalogSensor //


// Torque control
bool embObjMotionControl::setTorqueModeRaw()
{
    bool ret = true;
    eOmc_controlmode_command_t val = eomc_controlmode_cmd_torque;
    for(int j=0; j<_njoints; j++)
    {
        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);
        ret &= res->addSetMessage(protid, (uint8_t*) &val);
    }
    return ret;
}

bool embObjMotionControl::getTorqueRaw(int j, double *t)
{
    double NEWTON2SCALE=32768.0/_maxTorque[j];
    eOmeas_torque_t meas_torque;
    uint16_t size;
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_inputs_externallymeasuredtorque);
    bool ret = res->readSentValue(protoid, (uint8_t*) &meas_torque, &size);
    *t = ( (double) meas_torque / NEWTON2SCALE);
    return ret;
}

bool embObjMotionControl::getTorquesRaw(double *t)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && getTorqueRaw(j, &t[j]);
    return true;
}

bool embObjMotionControl::getTorqueRangeRaw(int j, double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getTorqueRangeRaw");
}

bool embObjMotionControl::getTorqueRangesRaw(double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getTorqueRangesRaw");
}

bool embObjMotionControl::setRefTorquesRaw(const double *t)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= setRefTorqueRaw(j, t[j]);
    return ret;
}

bool embObjMotionControl::setRefTorqueRaw(int j, double t)
{
    eOmc_setpoint_t setpoint;
    setpoint.type = (eOenum08_t) eomc_setpoint_torque;
    setpoint.to.torque.value =  (eOmeas_torque_t)t ;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    return res->addSetMessage(protid, (uint8_t*) &setpoint);
}

bool embObjMotionControl::getRefTorquesRaw(double *t)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= getRefTorqueRaw(j, &t[j]);
    return ret;
}

bool embObjMotionControl::getRefTorqueRaw(int j, double *t)
{
    bool ret;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    res->addGetMessage(protid);

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "getRefTorque timed out, joint " << j;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_setpoint_t mysetpoint;

    ret = res->readBufferedValue(protid, (uint8_t *)&mysetpoint, &size);
    *t = (double) mysetpoint.to.torque.value;
    return ret;
}

bool embObjMotionControl::setTorquePidRaw(int j, const Pid &pid)
{
    eOmc_PID_t  outPid;
    copyPid_iCub2eo(&pid, &outPid);
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidtorque);
    return res->addSetMessage(protid, (uint8_t *)&outPid);
}

bool embObjMotionControl::setTorquePidsRaw(const Pid *pids)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= setTorquePidRaw(j, pids[j]);
    return ret;
}

bool embObjMotionControl::getTorqueErrorRaw(int j, double *err)
{
    uint16_t size;
    bool ret = true;
    eOmc_joint_status_ofpid_t pid_status;
    int mycontrolMode;
    /* Values in pid.XXX fields are valid ONLY IF we are in the corresponding control mode.
    Read it from the signalled message so we are sure that mode and pid values are coherent to each other
    In realtà potrebbe arrivare un nuovo msg tra la lettura del controlmode e la lettura dello status del pid
    Approfondire!! TODO*/

    getControlModeRaw(j, &mycontrolMode);
    if(VOCAB_CM_TORQUE != mycontrolMode)
    {
        //yWarning() << "Asked for Torque PID Error while not in Torque control mode. Returning zeros";
        err = 0;
        return false;
    }
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_ofpid);

    ret = res->readBufferedValue(protid, (uint8_t *)&pid_status, &size);
    *err = (double) pid_status.error;
    return ret;
}

bool embObjMotionControl::getTorqueErrorsRaw(double *errs)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= getTorqueErrorRaw(j, &errs[j]);
    return ret;
}

bool embObjMotionControl::getTorquePidRaw(int j, Pid *pid)
{
	// EOnv tmp;
    //_mutex.wait();
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidtorque);

    // Sign up for waiting the reply FIRST OF ALL!!
    eoThreadEntry *tt = appendWaitRequest(j, protid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if(!res->addGetMessage(protid) )
        return false;

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "get Torque Pid timed out, joint " << j;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    eOmc_PID_t eoPID;
    bool ret = res->readBufferedValue(protid, (uint8_t *)&eoPID, &size);
    copyPid_eo2iCub(&eoPID, pid);
    return ret;
}

bool embObjMotionControl::getTorquePidsRaw(Pid *pids)
{
    // first set is done in the open function because the whole joint config is sent to the EMSs
    bool ret = true;
       // just one joint at time, wait for the answer before getting to the next.
    // This is because otherwise too many msg will be placed into EMS can queue
    for(int j=0, index=0; j<_njoints; j++, index++)
    {
        ret &=getTorquePidRaw(j, &pids[j]);
    }
    return ret;
}

bool embObjMotionControl::getImpedanceRaw(int j, double *stiffness, double *damping)
{
    // first set is done in the open function because the whole joint config is sent to the EMSs
    eOmc_impedance_t val;

    if(!getWholeImpedanceRaw(j, val))
        return false;

    *stiffness = (double) (val.stiffness * 0.001);
    *damping = (double) (val.damping * 0.001);
    return true;
}

bool embObjMotionControl::getWholeImpedanceRaw(int j, eOmc_impedance_t &imped)
{
    // first set is done in the open function because the whole joint config is sent to the EMSs
//    EOnv tmp;
    eOprotID32_t protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_impedance);

    // Sign up for waiting the reply
    eoThreadEntry *tt = appendWaitRequest(j, protoid);  // gestione errore e return di threadId, così non devo prenderlo nuovamente sotto in caso di timeout
    tt->setPending(1);

    if(!res->addGetMessage(protoid) )
        return false;

    // wait here
    if(-1 == tt->synch() )
    {
        int threadId;
        yError () << "getImpedance timed out, joint " << j << "for board " << _fId.boardNum;

        if(requestQueue->threadPool->getId(&threadId))
            requestQueue->cleanTimeouts(threadId);
        return false;
    }

    // Get the value
    uint16_t size;
    res->readBufferedValue(protoid, (uint8_t *)&imped, &size);
    return true;
}

bool embObjMotionControl::setImpedanceRaw(int j, double stiffness, double damping)
{
    bool ret = true;
    eOmc_impedance_t val;

    // Need to read the whole struct and modify just 2 of them -> now aching the old values and re-using them.
    // first set is done in the open function because the whole joint config is sent to the EMSs
    // cleaner solution, split the impedance structure into 2 separeted nework variables
//    if(!getWholeImpedanceRaw(j, val))
//        return false;

    // EMS will divide stiffness value by 1000 because the cycle is 1KHz. It is done on the EMS since it manage the cycle and knows the real Rate.
    _cacheImpedance[j].stiffness = (eOmeas_stiffness_t) (stiffness * 1000.0);
	_cacheImpedance[j].damping   = (eOmeas_damping_t) (damping * 1000.0);

	val.stiffness 	= _cacheImpedance[j].stiffness;
	val.damping 	= _cacheImpedance[j].damping;
	val.offset      = _cacheImpedance[j].offset;

    eOprotID32_t protid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_impedance);
    
#ifdef VERIFY_ROP_SETIMPEDANCE
    impedanceSignature[j]++;
    ret &= res->addSetMessageWithSignature(nvid, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (uint8_t *) &val, impedanceSignature[j]);
    return ret;
#endif

    ret &= res->addSetMessage(protid, (uint8_t *) &val);
    return ret;
}

bool embObjMotionControl::setImpedanceOffsetRaw(int j, double offset)
{
    bool ret = true;
    eOmc_impedance_t val;

    // first set is done in the open function because the whole joint config is sent to the EMSs
//    if(!getWholeImpedanceRaw(j, val))
//        return false;

	_cacheImpedance[j].offset   = (eOmeas_torque_t) offset;
	val.stiffness 	= _cacheImpedance[j].stiffness;
	val.damping 	= _cacheImpedance[j].damping;
    val.offset  	= _cacheImpedance[j].offset;
    
    eOprotID32_t protid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_impedance);
    
#ifdef VERIFY_ROP_SETIMPEDANCE
    impedanceSignature[j]++;
    ret &= res->addSetMessageWithSignature(nvid, (eOcfg_nvsEP_mc_endpoint_t)_fId.ep, (uint8_t *) &val, impedanceSignature[j]);
    return ret;
#endif

    ret &= res->addSetMessage(protid, (uint8_t *) &val);

    return ret;
}

bool embObjMotionControl::getImpedanceOffsetRaw(int j, double *offset)
{
    eOmc_impedance_t val;

    if(!getWholeImpedanceRaw(j, val))
        return false;

    *offset = val.offset;
    return true;
}

bool embObjMotionControl::getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    *min_stiff = _impedance_limits[j].min_stiff;
    *max_stiff = _impedance_limits[j].max_stiff;
    *min_damp  = _impedance_limits[j].min_damp;
    *max_damp  = _impedance_limits[j].max_damp;
    return true;
}

bool embObjMotionControl::getBemfParamRaw(int j, double *bemf)
{
    return NOT_YET_IMPLEMENTED("getBemfParam");
}

bool embObjMotionControl::setBemfParamRaw(int j, double bemf)
{
    return NOT_YET_IMPLEMENTED("getBemfParam");
}

bool embObjMotionControl::setTorqueErrorLimitRaw(int j, double limit)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitRaw");
}

bool embObjMotionControl::setTorqueErrorLimitsRaw(const double *limits)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimitsRaw");
}

bool embObjMotionControl::getTorquePidOutputRaw(int j, double *out)
{
    return NOT_YET_IMPLEMENTED("getTorquePidOutputRaw");
}

bool embObjMotionControl::getTorquePidOutputsRaw(double *outs)
{
    return NOT_YET_IMPLEMENTED("getTorquePidOutputsRaw");
}

bool embObjMotionControl::getTorqueErrorLimitRaw(int j, double *limit)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitRaw");
}

bool embObjMotionControl::getTorqueErrorLimitsRaw(double *limits)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimitsRaw");
}

bool embObjMotionControl::resetTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetTorquePidRaw");
}

bool embObjMotionControl::disableTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("disableTorquePidRaw");
}

bool embObjMotionControl::enableTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("enableTorquePidRaw");
}

bool embObjMotionControl::setTorqueOffsetRaw(int j, double v)
{
    return NOT_YET_IMPLEMENTED("setTorqueOffsetRaw");
}

// IVelocityControl2
bool embObjMotionControl::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    bool ret = true;

    for(int j=0; j< n_joint; j++)
    {
        ret &= velocityMoveRaw(joints[j], spds[j]);
    }
    return ret;
}

bool embObjMotionControl::setVelPidRaw(int j, const Pid &pid)
{
    // Our boards do not have a Velocity Pid
    return NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}

bool embObjMotionControl::setVelPidsRaw(const Pid *pids)
{
    // Our boards do not have a VelocityPid
    return NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}

bool embObjMotionControl::getVelPidRaw(int j, Pid *pid)
{
    // Our boards do not have a VelocityPid
    return NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}

bool embObjMotionControl::getVelPidsRaw(Pid *pids)
{
    // Our boards do not have a VelocityPid
    return NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}


// PositionDirect Interface
bool embObjMotionControl::setPositionRaw(int j, double ref)
{
    // needs to send both position and velocit as well as positionMove
    // does the same as setReferenceRaw, with some more misterious (missing) checks.
	return setReferenceRaw(j, ref);
}

bool embObjMotionControl::setPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    // needs to send both position and velocit as well as positionMove
    bool ret = true;    
    for(int i=0; i<n_joint; i++)
    {
        ret &= setReferenceRaw(joints[i], refs[i]);
    }
	return ret;
}

bool embObjMotionControl::setPositionsRaw(const double *refs)
{
    // needs to send both position and velocit as well as positionMove
	return setReferencesRaw(refs);
}


// eof



