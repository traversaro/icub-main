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

#ifndef ICUB_SKINMANAGER_CONTACTFORCETORQUEESTIMATION_H
#define ICUB_SKINMANAGER_CONTACTFORCETORQUEESTIMATION_H

#include <iCub/skinDynLib/skinContact.h>

namespace iCub{

namespace skinManager{

/**
* Interface for the estimators of the force/torque excerted by a contact.
*/
class IContactForceTorqueEstimator
{
public:
    /**
     * Destructor
     */
    virtual ~IContactForceTorqueEstimator();

    /**
     * Compute the contact force/torque from the raw output of the taxels
     * The estimated contact force/torque is stored in the skinContact class itself
     * @param contact the skinContact for which the force/torque is estimated
     */
    virtual void computeContactForceTorque(iCub::skinDynLib::skinContact& contact) = 0;
};

/**
* Class for basic estimation of the contact force/torque
*
* @note This is the estimation method hardcoded in the skinManager before the introduction of IContactForceTorqueEstimator
*/
class DummyContactForceTorqueEstimator: public IContactForceTorqueEstimator
{
public:
    DummyContactForceTorqueEstimator();
    virtual ~DummyContactForceTorqueEstimator();

    // Documented in IContactForceTorqueEstimator
    virtual void computeContactForceTorque(iCub::skinDynLib::skinContact& contact);
};


}

}

#endif

