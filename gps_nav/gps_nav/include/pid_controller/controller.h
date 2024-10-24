#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>

class PIDImpl;
class Controller
{
    public:
        Controller(double dt, double Kp, double Ki, double Kd);
        double calculate(double new_err);j
        ~Controller();

    private:
        PIDImpl *pidimpl;
}

#endif