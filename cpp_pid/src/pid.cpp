// gist.github.com/bradley219/5373998

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "cpp_pid/pid.h"

namespace auvsl
{
    // Constructor
    PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki) :
        _dt(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0)
    {
    }

    // Destructor
    PID::~PID()
    {
    }

    // compute the error
    double PID::calculate(double new_err)
    {   
        // Proportional term
        double Pout = _Kp*new_err;
        
        // Integral term
        _integral += new_err*_dt;
        double Iout = _Ki*_integral;

        // Derivative term
        double derivative = (new_err-_prev_error)/_dt;
        double Dout = _Kd*derivative;

        // Calculate total output
        double output = Pout + Iout + Dout;

        // Restrict to max/min
        if( output > _max )
            output = _max;
        else if( output < _min )
            output = _min;

        // Save error to previous error
        _prev_error = new_err;

        return output;
    }
}

#endif