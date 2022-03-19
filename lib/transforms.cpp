#include "transforms.h"
#include <iostream>
const float TR[2][3] = {
    {1.0, -0.5, -0.5},
    {0, sqrt(3.0)/2.0, -sqrt(3.0)/2.0}
};

const float gain = sqrt(2.0 / 3.0);

InstantPower InstantPower::operator+(const InstantPower &a){
    return InstantPower{
        this->P + a.P,
        this->Q + a.Q
    };
}

InstantPower InstantPower::operator-(const InstantPower &a){
    return InstantPower{
        this->P - a.P,
        this->Q - a.Q
    };
}

Normal::Normal(double a, double b, double c){
    this->a = a;
    this->b = b;
    this->c = c;
}

Normal Normal::operator+(const Normal &a){
  return Normal(
        this->a + a.a,
        this->b + a.b,
        this->c + a.c
    );
}

Normal Normal::operator-(const Normal &a){
  return Normal(
        this->a - a.a,
        this->b - a.b,
        this->c - a.c
    );
}

Clarke::Clarke(Normal n){
  this->alpha =
      gain * (TR[0][0] * n.a + TR[0][1] * n.b + TR[0][2] * n.c);
  this->beta = gain *
               (TR[1][0] * n.a + TR[1][1] * n.b + TR[1][2] * n.c);
}

Clarke::Clarke(double alpha, double beta){
    this->alpha=alpha;
    this->beta=beta;
}

Normal Clarke::to_normal(){
  return Normal(gain * (this->alpha * TR[0][0] + this->beta * TR[1][0]),
                gain * (this->alpha * TR[0][1] + this->beta * TR[1][1]),
                gain * (this->alpha * TR[0][2] + this->beta * TR[1][2]));
}

InstantPower Clarke::operator*(const Clarke &a) {
    return InstantPower{
      .P = this->alpha * a.alpha + this->beta * a.beta,
      .Q = -this->alpha * a.beta + this->beta * a.alpha
    };
}

Clarke Clarke::operator+(const Clarke &a){
    return Clarke(
        this->alpha + a.alpha,
        this->beta + a.beta
    );
}

Clarke Clarke::operator-(const Clarke &a){
    return Clarke(
        this->alpha - a.alpha,
        this->beta - a.beta
    );
}

Park::Park(Clarke c, double theta){
    this->d = cos(theta) * c.alpha + sin(theta) * c.beta;
    this->q = -sin(theta) * c.alpha + cos(theta) * c.beta;
}

Park::Park(Normal n, double theta) {
    Park p = Park(Clarke(n), theta);
    this->d = p.d;
    this->q = p.q;
}

Park::Park(double d, double q){
    this->d = d;
    this->q = q;
}


Clarke Park::to_clarke(double theta){
  return Clarke(cos(theta) * this->d - sin(theta) * this->q,
                sin(theta) * this->d + cos(theta) * this->q);
}

Normal Park::to_normal(double theta) {
    return this->to_clarke(theta).to_normal();
}

InstantPower Park::operator*(const Park &a) {
    return InstantPower{
        .P = this->d * a.d + this->q * a.q,
        .Q = -this->d * a.q + this->q * a.d,
    };
}

Park Park::operator+(const Park &a){
    return Park(
        this->d + a.d,
        this->q + a.q
    );
}

Park Park::operator-(const Park &a){
    return Park(
        this->d - a.d,
        this->q - a.q
    );
}

ThetaController::ThetaController(float Kp, float Ki, float T_samp) {
    const double PI = 3.141592;
    SatParam it = SatParam(
        SaturationType::none,
        PI,
        -PI
    );
    this->theta = Integrator(T_samp, it);
    this->ang_vel = PIController(Kp, Ki, T_samp);
}

double ThetaController::actuation(Park V){
    double angular_vel = this->ang_vel.actuation(V.q);
    return this->theta.actuation(angular_vel);
}
double ThetaController::read() { return this->theta.read(); }

double ThetaController::read_vel(){
    return this->ang_vel.read();
}


Park ParkController::actuation(Park obj){
    this->upper.actuation(obj.d);
    this->lower.actuation(obj.q);
    return this->read();
}




