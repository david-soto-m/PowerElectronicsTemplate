#include "transforms.h"
const float TR[2][3] = {
    {1.0, -0.5, -0.5},
    {0, sqrt(3.0)/2.0, -sqrt(3.0)/2.0}
};

const float gain = sqrt(2.0/3.0);

Clarke clarke_tr(Normal item){
  Clarke a = {
        .alpha = gain * (
            TR[0][0] * item.a
            + TR[0][1] * item.b
            + TR[0][2] * item.c),
        .beta = gain * (
            TR[1][0] * item.a
            + TR[1][1] * item.b
            + TR[1][2] * item.c)
    };
    return a;
}

Park park_tr(Normal item, float theta) {
    Clarke cl = clarke_tr(item);
    return clarke_to_park(cl, theta);
}
Park clarke_to_park(Clarke cl,float theta){
    Park a = {
        .d = cos(theta) * cl.alpha + sin(theta) * cl.beta,
        .q = -sin(theta) * cl.alpha + cos(theta) * cl.beta,
    };
    return a;
}

Normal clarke_inv_tr(Clarke item) {
    Normal a = {
        .a = gain * (
        item.alpha * TR[0][0]
        + item.beta * TR[1][0]),
        .b = gain * (
            item.alpha * TR[0][1]
            + item.beta * TR[1][1]),
        .c = gain * (
            item.alpha * TR[0][2]
            + item.beta * TR[1][2]),
    };
    return a;
}

Clarke clarke_from_Park(Park item, float theta) {
    Clarke a = {
        .alpha = cos(theta) * item.d - sin(theta) * item.q,
        .beta  = sin(theta) * item.d + cos(theta) * item.q,
    };
    return a;
}


Normal park_inv_tr(Park item, float theta) {
    Clarke cl = clarke_from_Park(item, theta);
    return clarke_inv_tr(cl);
}

InstantPower clarke_power(Clarke V, Clarke I){
    InstantPower a = {
        .P = V.alpha*I.alpha + V.beta*I.beta,
        .Q = -V.alpha*I.beta + V.beta*I.alpha
    };
    return a;
}

InstantPower park_power(Park V, Park I){
    InstantPower a = {
        .P = V.d*I.d+ V.q*I.q,
        .Q = -V.d*I.q+V.q*I.d,
    };
    return a;
}



ThetaController new_ThetaController(float Kp, float Ki, float T_samp) {
    const float PI = 3.141592;
    Saturator sat = {
        .sat = none,
        .max = PI,
        .min = -PI
    };
    ThetaController a = {
        .ang_vel = new_Controller(Kp,Ki,T_samp),
        .integ = new_Controller_w_sat(0, 0, T_samp, sat)
    };
    return a;
}

float th_ang(ThetaController theta) {
    return read(theta.integ);
}

float th_vel(ThetaController theta) {
    return read(theta.ang_vel);
}


float theta_ctrl(ThetaController *ctrl, Park V){
    float ang_vel = pi_simple(&ctrl->ang_vel, V.q);
    float ang = integ(&ctrl->integ, ang_vel);
    return ang;
}

ParkController new_ParkController(float Kp, float Ki, float T_samp){
    ParkController a = {
        .d = new_Controller(Kp, Ki, T_samp),
        .q = new_Controller(Kp, Ki, T_samp),
    };
    return a;
}

Park actuate_Park(ParkController *A, Park V) {
    Park a = {
        .d = pi_simple(&A->d, V.d),
        .q = pi_simple(&A->q, V.q),
    };
    return a;
}

Park read_Park(ParkController A){
    Park a ={
        .d =read(A.d),
        .q =read(A.q),
    };
    return a;
}
