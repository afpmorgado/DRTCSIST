#include "pid.h"
#include <iostream>


pid::pid(float _K, float _b, float _Ti, float _h)
    : K{_K}, b{_b}, Ti{_Ti}, h{_h}, I{0.0}, y_old{0.0},
      feedback{1}, antiWindup{1}, bumpless {1}, Tt{0.1},
      Kold{_K}, bold{_b}
{
}

float pid::computeControl(float r, float y)
{
    float FF, u, u_sat;
    FF = K*b*r; //feedforward term

    FF = saturate(FF, float(0), float(1));

    if (!feedback) { //if feedback isn't ON then simply return FF term
        return FF;
    }

    float bi = K * h / Ti; //used to calculate I term
    float ao = h / Tt; //used in antiwindup

    float e = r - y; //error
    float P = K * -y; //Proportional term

    if (bumpless == 1) //for seamless value transitions of K and b
    {
      I += Kold * (bold * r - y) - K * (b * r - y);
      Kold = K;
      bold = b;
    }

    I += bi * e; //I term

    u = FF + P + I; //u term

    u_sat = saturate(u, float(0), float(1));

    if (antiWindup) { //compensate for error accumulation in the integrator overtime
      I += ao * (u_sat - u);
    }
    y_old = y;

    return u_sat;
}

float pid::saturate(float value, float min_val, float max_val) //auxiliary function to saturate values
{
    if (value < min_val) {
        return min_val;
    }
    else if (value > max_val) {
        return max_val;
    }
    else {
        return value;
    }
}

//Auxiliary functions used for obtaining plots that demontrate the correct functioning of the PI controller

void pid::setK(float newK)
{
    K = newK;
}

float pid::getK()
{
    return K;
}

void pid::setB(float newB)
{
    b = newB;
}

float pid::getB()
{
    return b;
}

void pid::setTi(float newTi)
{
    Ti = newTi;
}

void pid::setFeedback(int value)
{
    feedback = value;
}

int pid::getFeedback()
{
    return feedback;
}

void pid::setAntiWindup(int value)
{
    antiWindup = value;
}

int pid::getAntiWindup()
{
    return antiWindup;
}

void pid::setBumpless(int value)
{
    bumpless = value;
}

int pid::getBumpless()
{
    return bumpless;
}