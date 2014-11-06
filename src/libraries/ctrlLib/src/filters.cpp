/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>

#include <yarp/math/Math.h>
#include <iCub/ctrl/filters.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

/***************************************************************************/
FIFOCircularBuffer::FIFOCircularBuffer()
{
    reset(0,yarp::sig::Vector());
}

/***************************************************************************/
FIFOCircularBuffer::FIFOCircularBuffer(const unsigned int size, const yarp::sig::Vector & vec)
{
    reset(size,vec);
}

/***************************************************************************/
void FIFOCircularBuffer::reset(const unsigned int size, const Vector& vec)
{
    front = 0;
    storage.resize(size);
    for (size_t i=0; i<storage.size(); i++)
        storage[i] = vec;
}

/***************************************************************************/
void FIFOCircularBuffer::insert(const yarp::sig::Vector & vec)
{
    front++;
    front = front % storage.size();
    storage[front] = vec;
}

/***************************************************************************/
unsigned int FIFOCircularBuffer::size() const
{
    return storage.size();
}


/***************************************************************************/
yarp::sig::Vector & FIFOCircularBuffer::operator[](int i)
{
    int internal_index = (front-i+storage.size())%storage.size();
    return storage[internal_index];
}


/***************************************************************************/
Filter::Filter(const Vector &num, const Vector &den, const Vector &y0)
{
    b=num;
    a=den;

    m=b.length();
    n=a.length();

    init(y0);
}


/***************************************************************************/
void Filter::init(const Vector &y0)
{
    // if there is a last input then take it as guess for the next input
    if (uold.size() > 0 && uold[0].size() == y0.size())
        init(y0,uold[0]);
    else    // otherwise use zero
        init(y0,zeros(y0.length()));
}


/***************************************************************************/
void Filter::init(const Vector &y0, const Vector &u0)
{
    Vector u_init(y0.length(),0.0);
    Vector y_init=y0;
    y=y0;

    double sum_b=0.0;
    for (size_t i=0; i<b.length(); i++)
        sum_b+=b[i];

    double sum_a=0.0;
    for (size_t i=0; i<a.length(); i++)
        sum_a+=a[i];

    if (fabs(sum_b)>1e-9)   // if filter DC gain is not zero
        u_init=(sum_a/sum_b)*y0;
    else
    {
        // if filter gain is zero then you need to know in advance what
        // the next input is going to be for initializing (that is u0)
        // Note that, unless y0=0, the filter output is not going to be stable
        u_init=u0;
        if (fabs(sum_a-a[0])>1e-9)
            y_init=a[0]/(a[0]-sum_a)*y;
        // if sum_a==a[0] then the filter can only be initialized to zero
    }

    //for (size_t i=0; i<yold.size(); i++)
    //    yold[i]=y_init;
    yold.reset(n-1,y_init);

    //for (size_t i=0; i<uold.size(); i++)
    //    uold[i]=u_init;
    uold.reset(m-1,u_init);

}


/***************************************************************************/
void Filter::getCoeffs(Vector &num, Vector &den)
{
    num=b;
    den=a;
}


/***************************************************************************/
void Filter::setCoeffs(const Vector &num, const Vector &den)
{
    b=num;
    a=den;

    m=b.length();
    n=a.length();

    init(y);
}


/***************************************************************************/
bool Filter::adjustCoeffs(const Vector &num, const Vector &den)
{
    if ((num.length()==b.length()) && (den.length()==a.length()))
    {
        b=num;
        a=den;

        return true;
    }
    else
        return false;
}

/***************************************************************************/
/**
* Add alpha*X to Y. Equivalent to Y += alpha*X, but avoids memory allocation.
*
*/
void VectorAccumulate(double alpha,
                      const yarp::sig::Vector & X,
                      yarp::sig::Vector & Y)
{
    gsl_blas_daxpy(alpha,
                   (const gsl_vector*)X.getGslVector(),
                   (gsl_vector*)Y.getGslVector());
}

/***************************************************************************/
const Vector& Filter::filt(const Vector &u)
{
    //y=b[0]*u;
    y.zero();
    VectorAccumulate(b[0],u,y);

    for (size_t i=1; i<m; i++)
    {
        //y+=b[i]*uold[i-1];
        VectorAccumulate(b[i],uold[i-1],y);
    }

    for (size_t i=1; i<n; i++)
    {
        //y-=a[i]*yold[i-1];
        VectorAccumulate(-a[i],yold[i-1],y);
    }

    //y=(1.0/a[0])*y;
    y *= (1.0/a[0]);

    //uold.push_front(u);
    //uold.pop_back();
    uold.insert(u);

    //yold.push_front(y);
    //yold.pop_back();
    yold.insert(y);


    return y;
}


/**********************************************************************/
RateLimiter::RateLimiter(const Vector &rL, const Vector &rU) :
                         rateLowerLim(rL), rateUpperLim(rU)
{
    size_t nL=rateLowerLim.length();
    size_t nU=rateUpperLim.length();

    n=nU>nL ? nL : nU;
}


/**********************************************************************/
void RateLimiter::init(const Vector &u0)
{
    uLim=u0;
}


/**********************************************************************/
void RateLimiter::getLimits(Vector &rL, Vector &rU)
{
    rL=rateLowerLim;
    rU=rateUpperLim;
}


/**********************************************************************/
void RateLimiter::setLimits(const Vector &rL, const Vector &rU)
{
    rateLowerLim=rL;
    rateUpperLim=rU;
}


/**********************************************************************/
Vector RateLimiter::filt(const Vector &u)
{
    uD=u-uLim;
    for (size_t i=0; i<n; i++)
    {
        if (uD[i]>rateUpperLim[i])
            uD[i]=rateUpperLim[i];
        else if (uD[i]<rateLowerLim[i])
            uD[i]=rateLowerLim[i];
    }

    uLim+=uD;

    return uLim;
}


/**********************************************************************/
FirstOrderLowPassFilter::FirstOrderLowPassFilter(const double cutFrequency,
                                                 const double sampleTime,
                                                 const Vector &y0)
{
    fc = cutFrequency;
    Ts = sampleTime;
    y = y0;
    filter = NULL;
    computeCoeff();
}


/**********************************************************************/
FirstOrderLowPassFilter::~FirstOrderLowPassFilter()
{
    if(filter!=NULL)
        delete filter;
}


/***************************************************************************/
void FirstOrderLowPassFilter::init(const Vector &y0)
{
    if(filter!=NULL)
        filter->init(y0);
}


/**********************************************************************/
bool FirstOrderLowPassFilter::setCutFrequency(const double cutFrequency)
{
    if(cutFrequency<=0.0)
        return false;
    fc = cutFrequency;
    computeCoeff();
    return true;
}


/**********************************************************************/
bool FirstOrderLowPassFilter::setSampleTime(const double sampleTime)
{
    if(sampleTime<=0.0)
        return false;
    Ts = sampleTime;
    computeCoeff();
    return true;
}


/**********************************************************************/
const Vector& FirstOrderLowPassFilter::filt(const Vector &u)
{
    if(filter!=NULL)
    {
        y = filter->filt(u);
        //If the filter is workin
        //return directly a reference to the underlyng
        //filter output, to avoid a Vector constructor
        //call and the relative memory allocation
        return filter->output();
    }
    return y;
}


/**********************************************************************/
void FirstOrderLowPassFilter::computeCoeff()
{
    double tau = 1.0/(2.0*M_PI*fc);
    Vector num = cat(Ts, Ts);
    Vector den = cat(2.0*tau+Ts, Ts-2.0*tau);
    if(filter!=NULL)
        filter->adjustCoeffs(num, den);
    else
        filter = new Filter(num, den, y);
}


