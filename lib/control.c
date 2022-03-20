#include "control.h"

void saturate(Controller *I){
    switch (I->sat.sat){
        case circular:
            if(I->res > I->sat.max){
                I->res = I->sat.min;
            }else if(I->res < I->sat.min){
                I->res = I->sat.max;
            }
            break;
        case roof:
            if(I->res > I->sat.max){
                I->res = I->sat.max;
            }else if(I->res < I->sat.min){
                I->res = I->sat.min;
            }
            break;
        default:
            break;
    }
}


float pi_simple(Controller *C, float var) {
    C->res += C->Kp * var + (C->Ki * C->T_samp - C->Kp) * C->var;
    C->var = var;
    saturate(C);
    return C->res;
}


float integ(Controller *I, float var) {
    I->res += var * I->T_samp;
    saturate(I);
    return I->res;
}

float read(Controller a) { return a.res; }

Controller new_Controller(float Kp, float Ki, float T_samp) {
      Controller a = {
        .Kp = Kp,
        .Ki = Ki,
        .T_samp = T_samp,
        .res = 0,
        .var = 0,
        .sat = {
            .sat = none,
            .max = 0,
            .min = 0
        }
    };
    return a ;
}


Controller new_Controller_w_sat(float Kp,
                                float Ki,
                                float T_samp,
                                Saturator sat) {
    Controller a = {
        .Kp = Kp,
        .Ki = Ki,
        .T_samp = T_samp,
        .res = 0,
        .var = 0,
        .sat = sat
    };
    return a;}
