#ifndef TRANSFORMS_H
#define TRANSFORMS_H
#include <math.h>
#include "control.h"


// BEGIN Types


/*
 * Represents a direct electric measurement
 */
typedef struct Normal {
    float a;
    float b;
    float c;
} Normal;

/*
 * Represents a measurement in the Clarke space
 */
typedef struct Clarke {
    float alpha;
    float beta;
} Clarke;

/*
 * Represents a measurement in the Park space
 */
typedef struct Park {
    float d;
    float q;
} Park;


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
 * get angle from ThetaResult Structure
 */
float th_ang(ThetaResult theta);

/*
 * get the angular velocity form the ThetaResult Structure
 */
float th_vel(ThetaResult theta);

/*
 * Return the clarke transform
 */
Clarke clarke_tr(Normal item);

/*
 * Return a Park space item form a clarke
 */
Park clarke_to_park(Clarke item, float theta);

/*
 * Return a Park item
 */
Park park_tr(Normal item, float theta);

/*
 * return an inverted clarke transform
 */
Normal clarke_inv_tr(Clarke item);

Clarke clarke_from_Park(Park item, float theta);
/*
 * return an inverted from park transform, includes clarke transform
 */
Normal park_inv_tr(Park item, float theta);

/*
 * PLL and integrator blocks in one function that uses the special ControlResult
 * ThetaResult.
 * Theta is accessible at var.integ.res
 */
ThetaResult theta_ctrl(Park V, const Controler c, ThetaResult past);

/*
 * Calculate the power from clarke transform
 */
InstantPower clarke_power(Clarke V, Clarke I);

/*
 * Calculate the power from the park transform
 */
InstantPower park_power(Park V, Park I);

//END Methods
#endif  // TRANSFORMS_H

