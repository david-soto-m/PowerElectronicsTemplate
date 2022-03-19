#include "control.h"

PIController::PIController(){
    this->Ki = 0;
    this->Kp = 0;
    this->T_samp = 0;
    this->sat = SatParam();
    this->res = 0;
    this->var = 0;
}
PIController::PIController(double Kp,
                           double Ki,
                           double T_samp,
                           SatParam sat) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->T_samp = T_samp;
    this->sat = sat;
    this->res = 0;
    this->var = 0;
}
PIController::PIController(double Kp, double Ki, double T_samp) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->T_samp = T_samp;
    this->sat = SatParam();
    this->res = 0;
    this->var = 0;
}
void PIController::saturate() {
    switch (this->sat.type){
        case SaturationType::circular:
            if(this->res > this->sat.max){
                this->res = this->sat.min;
            }else if(this->res < this->sat.min){
                this->res = this->sat.max;
            }
            break;
        case SaturationType::roof:
            if(this->res > this->sat.max){
                this->res = this->sat.max;
            }else if(this->res < this->sat.min){
                this->res = this->sat.min;
            }
            break;
        default:
            break;
    }
}
double PIController::actuation(double var){
    this->res += this->Kp * var
                 + (this->Ki * this->T_samp - this->Kp) * this->var;
    this->var = var;
    this->saturate();
    return this->res;
}
double PIController::read() { return this->res; }



Integrator::Integrator(){
    this->Ki = 0;
    this->Kp = 0;
    this->T_samp = 0;
    this->sat = SatParam();
    this->res = 0;
    this->var = 0;
}

Integrator::Integrator(double Tsamp, SatParam sat) {
    this->Ki = 0;
    this->Kp = 0;
    this->T_samp = Tsamp;
    this->sat = sat;
    this->res = 0;
    this->var = 0;
}

double Integrator::actuation(double var){
    this->res += var * this->T_samp;
    saturate();
    return this->res;
}
