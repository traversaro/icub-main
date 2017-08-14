/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#include <iCub/skinManager/ContactForceTorqueEstimation.h>

// Necessary for the operator*(double ..., yarp::sig::Vector ...) overload
#include <yarp/math/Math.h>
using namespace yarp::math;

namespace iCub
{
namespace skinManager
{

IContactForceTorqueEstimator::~IContactForceTorqueEstimator()
{
}

DummyContactForceTorqueEstimator::DummyContactForceTorqueEstimator()
{
}

DummyContactForceTorqueEstimator::~DummyContactForceTorqueEstimator()
{
}

void DummyContactForceTorqueEstimator::computeContactForceTorque(iCub::skinDynLib::skinContact &contact)
{
    // set an estimate of the force that is with normal direction and intensity equal to the pressure
    double pressure = contact.getPressure();
    unsigned int activeTaxels = contact.getActiveTaxels();
    contact.setForce(-0.05*activeTaxels*pressure*contact.getNormalDir());
    return;
}



}
}