#include "transforms.h"
const float TR[2][3] = {
    {1.0, -0.5, -0.5},
    {0, sqrt(3.0)/2.0, -sqrt(3.0)/2.0}
};

const float gain = sqrt(2.0/3.0);

ClarkePark clarke_tr(ClarkePark item){
    item.alpha = gain * (
        TR[0][0] * item.a
        + TR[0][1] * item.b
        + TR[0][2] * item.c);
    item.beta = gain * (
        TR[1][0] * item.a
        + TR[1][1] * item.b
        + TR[1][2] * item.c);
    return item;
}

ClarkePark park_tr(ClarkePark item, float theta) {
    item = clarke_tr(item);
    item.d = cos(theta) * item.alpha + sin(theta) * item.beta;
    item.q = -sin(theta) * item.alpha + cos(theta) * item.beta;
    return item;
}

ClarkePark clarke_inv_tr(ClarkePark item) {
    item.a = gain * (
        item.alpha * TR[0][0]
        + item.beta * TR[1][0]);
    item.b = gain * (
        item.alpha * TR[0][1]
        + item.beta * TR[1][1]);
    item.c = gain * (
        item.alpha * TR[0][2]
        + item.beta * TR[1][2]);
    return item;
}


ClarkePark park_inv_tr(ClarkePark item, float theta) {
    item.alpha = cos(theta) * item.d - sin(theta) * item.q;
    item.beta  = sin(theta) * item.d + cos(theta) * item.q;
    return clarke_inv_tr(item);
}

ThetaResult new_ThetaResult() {
  ThetaResult a = {
        .ang_vel = new_ControlResult(),
        .integ = new_ControlResult()
    };
    return a;
}

ControlResult a;


ThetaResult theta_ctrl(ClarkePark V, const Controler C, ThetaResult past){
    past.ang_vel = pi_simple(V.q, C, past.ang_vel);

    const float PI = 3.141592;
    const Controler it = {
        .T_samp = C.T_samp,
        .sat = NONE,
        .max = PI,
        .min = -PI
    };
    past.integ = integ(past.ang_vel.res, it, past.integ);
    return past;
}

InstantPower clarke_power(ClarkePark V, ClarkePark I){
    InstantPower a = {
        .P = V.alpha*I.alpha + V.beta*I.beta,
        .Q = -V.alpha*I.beta + V.beta*I.alpha
    };
    return a;
}

InstantPower park_power(ClarkePark V, ClarkePark I){
    InstantPower a = {
        .P = V.d*I.d+ V.q*I.q,
        .Q = -V.d*I.q+V.q*I.d,
    };
    return a;
}
