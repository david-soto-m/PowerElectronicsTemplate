#include "control.h"

ControlResult saturate(const Controler I, ControlResult past){
    switch (I.sat){
        case CIRCULAR:
            if(past.res > I.max){
                past.res = I.min;
            }else if(past.res < I.min){
                past.res = I.max;
            }
            break;
        case ROOF:
            if(past.res > I.max){
                past.res = I.max;
            }else if(past.res < I.min){
                past.res = I.min;
            }
            break;
        default:
            break;
    }
    return past;
}


ControlResult pi_simple(float var, const Controler C, ControlResult past) {
    past.res += C.Kp * var + (C.Ki * C.T_samp - C.Kp) * past.var;
    past.var = var;
    return saturate(C, past);
}


ControlResult integ(float var, const Controler I, ControlResult past) {
    past.res += var * I.T_samp;
    return saturate(I, past);
}


ControlResult new_ControlResult(){
    ControlResult a = {.res = 0, .var=0};
    return a;
}

float actuation(ControlResult a) { return a.res; }


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
PIController::PIController(){
  this->Ki = 0;
  this->Kp = 0;
  this->T_samp = 0;
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

double Integrator::actuation(double var){
    this->res += var * this->T_samp;
    saturate();
    return this->res;
}

Integrator::Integrator(double Tsamp, SatParam sat) {
    PIController(0,0,Tsamp,sat);
}
