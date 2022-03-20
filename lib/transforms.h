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
 * Structure of computed powers
 */
typedef struct InstantPower{
    float P;
    float Q;
} InstantPower;

// END Types

// BEGIN Methods

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
 * Calculate the power from clarke transform
 */
InstantPower clarke_power(Clarke V, Clarke I);

/*
 * Calculate the power from the park transform
 */
InstantPower park_power(Park V, Park I);
// END Methods

// BEGIN ThetaController

/*
 * Special controller for two controls at a time, one the PLL and the other an
 * integrator.
 */
typedef struct ThetaController{
    Controller ang_vel;
    Controller integ;
} ThetaController;

/*
 * PLL and integrator blocks in one function
 */
float theta_ctrl(ThetaController *ctrl, Park V);

/*
 * Create an empty ThetaResult;
 */
ThetaController new_ThetaController(float Kp, float Ki, float T_samp);

/*
 * get angle from ThetaResult Structure
 */
float th_ang(ThetaController theta);

/*
 * get the angular velocity form the ThetaResult Structure
 */
float th_vel(ThetaController theta);


// END ThetaController


// BEGIN ParkController

typedef struct ParkController{
    Controller d;
    Controller q;
}  ParkController;

ParkController new_ParkController(float Kp, float Ki, float T_samp);

Park actuate_Park(ParkController *A, Park V);

Park read_Park(ParkController A);

// END Controller

#endif  // TRANSFORMS_H

