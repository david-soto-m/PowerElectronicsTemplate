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

float actuation(ControlResult a){
    return a.res;
}
