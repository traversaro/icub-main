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

#include <yarp/os/LogStream.h>

namespace iCub
{
namespace skinManager
{

IContactForceTorqueEstimator::~IContactForceTorqueEstimator()
{
}

DummyContactForceTorqueEstimator::DummyContactForceTorqueEstimator()
{
    forceTorqueEstimateConfidence=0;
}

DummyContactForceTorqueEstimator::~DummyContactForceTorqueEstimator()
{
}

bool DummyContactForceTorqueEstimator::open(yarp::os::Searchable&,
                                            const std::vector<yarp::sig::Vector>& /*taxelOrigins*/,
                                            const std::vector<yarp::sig::Vector>& /*taxelNormals*/)

{
    return true;
}

void DummyContactForceTorqueEstimator::computeContactForceTorque(iCub::skinDynLib::skinContact &contact,
                                                                 const std::vector<yarp::sig::Vector>& /*taxelOrigins*/,
                                                                 const std::vector<yarp::sig::Vector>& /*taxelNormals*/,
                                                                 const yarp::sig::Vector& /*rawTaxelData*/,
                                                                 const yarp::sig::Vector& /*compensatedTaxelData*/)
{
    // set an estimate of the force that is with normal direction and intensity equal to the pressure
    double pressure = contact.getPressure();
    unsigned int activeTaxels = contact.getActiveTaxels();
    contact.setForce(-0.05*activeTaxels*pressure*contact.getNormalDir());
    //set forceTorqueEstimateConfidence should be 0 -> no confidence
    contact.setForceTorqueEstimateConfidence(forceTorqueEstimateConfidence);
    return;
}


PolynomialTaxelCalibrationNoInterpolation::PolynomialTaxelCalibrationNoInterpolation()
{
    forceTorqueEstimateConfidence=1;
}

PolynomialTaxelCalibrationNoInterpolation::~PolynomialTaxelCalibrationNoInterpolation()
{
}

bool PolynomialTaxelCalibrationNoInterpolation::open(yarp::os::Searchable& config,
                                                     const std::vector<yarp::sig::Vector>& taxelOrigins,
                                                     const std::vector<yarp::sig::Vector>& taxelNormals)
{
    // Check for the polynomialOrder
    if ( !(config.check("polynomialOrder")) || (!config.find("polynomialOrder").isInt()) )
    {
        yError() << "PolynomialTaxelCalibrationNoInterpolation: impossible to find required integer parameter polynomialOrder";
        return false;
    }

    int signedPolynomialOrder = config.find("polynomialOrder").asInt();

    if (signedPolynomialOrder < 0)
    {
        yError() << "PolynomialTaxelCalibrationNoInterpolation: polynomialOrder needs to be nonnegative";
        return false;
    }

    polynomialOrder = signedPolynomialOrder;

    if ( !(config.check("taxelAreaInSquaredMeters")) || (!config.find("taxelAreaInSquaredMeters").isDouble()) )
    {
        yError() << "PolynomialTaxelCalibrationNoInterpolation: impossible to find required floating point parameter taxelAreaInSquaredMeters";
        return false;
    }

    taxelAreaInSquaredMeters = config.find("taxelAreaInSquaredMeters").asDouble();

    if (taxelAreaInSquaredMeters < 0)
    {
        yError() << "PolynomialTaxelCalibrationNoInterpolation: taxelAreaInSquaredMeters needs to be nonnegative";
        return false;
    }

    // Check the polyonomial parameters
    yarp::os::Bottle &calibration = config.findGroup("ContactForceTorqueEstimatorParameters");
    if (calibration.isNull())
    {
        yError() << "PolynomialTaxelCalibrationNoInterpolation: impossible to find required group ContactForceTorqueEstimatorParameters";
        return false;
    }
    else
    {
        int taxelCalibrationLines = calibration.size()-1;

        if (taxelCalibrationLines != taxelOrigins.size())
        {
            yError() << "PolynomialTaxelCalibrationNoInterpolation: error in size in ContactForceTorqueEstimatorParameters group";
            return false;
        }

        polynomialCoeffients.resize(taxelCalibrationLines, zeros(polynomialOrder+1));

        for (int i = 0; i < taxelCalibrationLines; ++i)
        {
            polynomialCoeffients[i] = iCub::skinDynLib::vectorFromBottle(*(calibration.get(i + 1).asList()), 0, polynomialOrder+1);
        }
    }

    return true;
}

void PolynomialTaxelCalibrationNoInterpolation::computeContactForceTorque(iCub::skinDynLib::skinContact &contact,
                                                                 const std::vector<yarp::sig::Vector>& taxelOrigins,
                                                                 const std::vector<yarp::sig::Vector>& taxelNormals,
                                                                 const yarp::sig::Vector& rawTaxelData,
                                                                 const yarp::sig::Vector& compensatedTaxelData)
{
    // Result of the calibration
    yarp::sig::Vector totalForce(3, 0.0), taxelForce(3, 0.0);
    yarp::sig::Vector totalTorque(3, 0.0), taxelTorque(3, 0.0);
    // yError()<<"Starting contact estimate" ;
    // double force=0;

    // Iterate on all taxels present in the contact (this should already exclude temperature taxels)
    taxelListBuffer = contact.getTaxelList();
    for (std::vector<unsigned int>::iterator it = taxelListBuffer.begin(); it != taxelListBuffer.end(); ++it)
    {
        // Get the taxelId
        unsigned int taxelId = *it;

        // Compute the calibrated pressure
        double taxelPressure = 0.0;
        double kThPowerOfRawOutput = 1.0;
        for (int k = 0; k <= polynomialOrder; ++k, kThPowerOfRawOutput = kThPowerOfRawOutput * (255-rawTaxelData[taxelId]))
        {
            // Polynomial coefficient are stored from the higher to the lower
            // There are alternative algorithms for evaluating polynomials that
            // introduce less numerical error, but for now use this formula for
            // consistency with the one used for estimation
            taxelPressure += kThPowerOfRawOutput * (polynomialCoeffients[taxelId][polynomialOrder - k]);
        }
        // yError()<<"Taxel Pressure"<<taxelPressure<<" rawTaxelData "<< rawTaxelData[taxelId] << "inverted rawTaxelData" <<(255-rawTaxelData[taxelId]) ;

        // As all complex formulas involving yarp::sig::Vector object, this call involve
        // a lot of dynamic memory allocation, and could be a performance bottleneck

        // Compute the force
        taxelForce = (taxelPressure*taxelAreaInSquaredMeters*1000)*taxelNormals[taxelId];
        totalForce += taxelForce;
        //force+=(taxelPressure*taxelAreaInSquaredMeters*1000);

        // Compute the torque (that is expressed with respect to the "center" of the contact)
        // TODO(traversaro, fjandrad): check the sign of the first term
        taxelTorque = cross(taxelOrigins[taxelId] - contact.getCoP(), totalForce);
        totalTorque += taxelTorque;
    }
    // yError()<<"Taxel Force"<<totalForce.toString()<<" Force magnitude "<<force;

    // Store the estimation result
    contact.setForceMoment(totalForce, totalTorque);
    // set the forceTorqueEstimateConfidence should be 1 -> a min of confidence
    contact.setForceTorqueEstimateConfidence(forceTorqueEstimateConfidence);
    return;
}

}
}
