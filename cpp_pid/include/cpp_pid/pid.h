// gist.github.com/bradley219/5373998

#ifndef _PID_H_
#define _PID_H_
namespace auvsl
{
    class PID
    {
        public:
            // Contructor and destructor
            PID(double dt, double max, double min, double Kp, double Ki, double Kd);
            ~PID();

            // Returns the manipulated variable given a setpoint and current process value
            double calculate(double err);
        private:
            double _dt;         // time step        
            double _max;        // maximum value of manipulated variable
            double _min;        // minimum value of manipulated variable
            double _Kp;         // proportional gain
            double _Ki;         // integral gain
            double _Kd;         // derivative gain
            double _prev_error; //error from last time step
            double _integral;   // running integrated error
    };
}
#endif