#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.hpp"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double err );

    private:
        double dt;
        double max;
        double min;
        double Kp;
        double Kd;
        double Ki;
        double alpha;
        double prev_err_;
        double integral_;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate(double err )
{
    return pimpl->calculate(err);
}
PID::~PID() 
{
    delete pimpl;
}


inline bool sign(double val) { return val > 0 ? true : false; }

/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    dt(dt),
    Kp(Kp),
    Kd(Kd),
    Ki(Ki),
    max(max),
    min(min),
    prev_err_(0),
    integral_(0),
    alpha(0.5)
{
}

double PIDImpl::calculate( double err )
{


    float ef = alpha * err + (1 - alpha) * prev_err_;

    // Integral term
    integral_ += err * dt;
    // Intergral windup avoidance
    // TODO: replace this with advanced algo 
    integral_ = std::min(integral_, 0.05);
    integral_ = std::max(integral_, 0.05);
    if (sign(err) != sign(integral_))
    {
        integral_ = 0; 
    }

    // Derivative term
    double diff = (err - prev_err_) / dt;

    // Calculate total output
    // double output = Kp * err + Ki * integral_ + Kd * diff;
    double output = Kp * ef + Kd * diff;
    if (err < 0.005 && err > -0.005)
        output += Ki * integral_; 

    // Restrict to max/min
    output = std::min(output, max);
    output = std::max(output, min); 

    // Save err to previous err
    prev_err_ = err;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
