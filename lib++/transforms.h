#ifndef TRANSFORMS_H
#define TRANSFORMS_H
#include <math.h>
#include "control.h"

// BEGIN Types
/*
 * Structure of computed powers
 */
struct InstantPower{
    double P;
    double Q;
    InstantPower operator+(const InstantPower &a);
    InstantPower operator-(const InstantPower &a);
};

/*
 * Represents a direct electric measurement
 */
class Normal {
public:
    double a;
    double b;
    double c;
    Normal(double a, double b, double c);
    Normal operator+(const Normal &a);
    Normal operator-(const Normal &a);
};

/*
 * Represents a measurement in the Clarke space
 */
class Clarke {
public:
    double alpha;
    double beta;
    Clarke(Normal n);
    Clarke(double alpha, double beta);
    Normal to_normal();
    InstantPower operator*(const Clarke &a);
    Clarke operator+(const Clarke &a);
    Clarke operator-(const Clarke &a);
};

/*
 * Represents a measurement in the Park space
 */
class Park{
public:
    double d;
    double q;
    Park(Normal n, double theta);
    Park(Clarke c, double theta);
    Park(double d, double q);
    Clarke to_clarke(double theta);
    Normal to_normal(double theta);
    InstantPower operator*(const Park &a);
    Park operator+(const Park &a);
    Park operator-(const Park &a);
};


/*
 * A controller for Park variables
 */
class ParkController : public DualController<Park, Park> {
public:
    Park actuation(Park obj);
    ParkController(float Kp, float Ki, float T_samp){
        this->upper = PIController(Kp, Ki, T_samp);
        this->lower = PIController(Kp, Ki, T_samp);
    }
    ParkController(float Kp, float Ki,float Kp_2, float Ki_2, float T_samp){
        this->upper = PIController(Kp, Ki, T_samp);
        this->lower = PIController(Kp_2, Ki_2, T_samp);
    }
};


/*
 * A controller for the theta variable with the integrator already included
 */
class ThetaController:SimpleController<double, Park>{
private:
    PIController ang_vel;
    Integrator theta;
public:
    ThetaController(float Kp, float Ki, float T_samp);
    double read();
    double read_vel();
    double actuation(Park V);
};

#endif // TRANSFORMS_H
