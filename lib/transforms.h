#ifndef TRANSFORMS_H
#define TRANSFORMS_H
#include <math.h>
#include "control.h"


// BEGIN Types
/* This is a Structure that holds all the ways an electric magnitude can be
 * represented
 *
 * This would be bad if it was in Rust.
 * The transforms would change between types and you would be able to do in
 * place transformations. Power would be a trait to be implemented or an
 * overloading of the product.
 */
typedef struct ClarkePark{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
    float d;
    float q;
} ClarkePark;

/*
 * Special controller for two controls at a time, one the PLL and the other an
 * integrator.
 */
typedef struct ThetaResult{
    ControlResult ang_vel;
    ControlResult integ;
} ThetaResult;

/*
 * Structure of computed powers
 */
typedef struct InstantPower{
    float P;
    float Q;
} InstantPower;

// END Types

// BEGIN Methods

/*
 * Create an empty ThetaResult;
 */
ThetaResult new_ThetaResult();

/*
 * Return the clarke transform
 */
ClarkePark clarke_tr(ClarkePark item);

/*
 * Return the park and clarke transforms
 */
ClarkePark park_tr(ClarkePark item, float theta);


/*
 * return an inverted clarke transform
 */
ClarkePark clarke_inv_tr(ClarkePark item);

/*
 * return an inverted from park transform, includes clarke transform
 */
ClarkePark park_inv_tr(ClarkePark item, float theta);

/*
 * PLL and integrator blocks in one function that uses the special ControlResult
 * ThetaResult.
 * Theta is accessible at var.integ.res
 */
ThetaResult theta_ctrl(ClarkePark V, const Controler c, ThetaResult past);

/*
 * Calculate the power from clarke transform
 */
InstantPower clarke_power(ClarkePark V, ClarkePark I);

/*
 * Calculate the power from the park transform
 */
InstantPower park_power(ClarkePark V, ClarkePark I);

//END Methods
#endif  // TRANSFORMS_H

