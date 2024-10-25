#include "pid_controller/controller.h"

namespace auvsl
{
    class PIDImpl
    {
        public:
            PIDImpl(double dt, double Kp, double Ki, double Kd);
            ~PIDImpl();
            double calculate(double err);
        private:
            double _dt;
            double _Kp;
            double _Ki;
            double _Kd;
            double _prev_error;
            double _integral;
    }



    // constructor definition
    Controller::Controller(double dt, double Kp, double Ki, double Kd)
    {
        pidimpl = new PIDImpl(dt, Kp, Ki, Kd);
    }

    // destructor definition
    Controller::~Controller()
    {
        delete pidimpl;
    }

    double Controller::calculate(err)
    {
        return pidimpl->calculate(err);
    }


    PIDImpl::PIDImpl(double dt, double Kp, double Ki, double Kd) :
        _dt(dt);
        _Kp(Kp);
        _Ki(Ki);
        _Kd(Kd);
        _prev_error(0);
        _integral(0);
    {
    }
    // gist.github.com/bradley219/5373998

    double PIDImpl::calculate(double new_err)
    {
        double Pout = _Kp*new_err;

        _integral += new_err*_dt;
        double Iout = _Ki*_integral;

        double derivative = (new_err-_prev_error)/_dt;
        double Dout = _Kd*derivative;

        double output = Pout + Iout + Dout;

        if (output > 31.74)
            output = 31.74;
        else if (output < -31.25)
            output = -31.25;

        _prev_error = new_err;

        return output;
    }

    PIDImpl::~PIDImpl()
    {
    }


}