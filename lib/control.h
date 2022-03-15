#ifndef CONTROL_H
#define CONTROL_H

// BEGIN types

/*
 * Parameters for a PI controller
 */
typedef struct Controler{
    float Kp;
    float Ki;
    float T_samp;
    char sat;
    float max;
    float min;

} Controler;

/*
 * A structure that holds the old control values that are to be remembered in
 * the next iteration. ***DO NOT MESS WITH IT***
 */
typedef struct ControlResult {
    float res;
    float var;
}ControlResult;

/*
 * A macro that creates an initialized to zero ControlResult structure in a
 * declarative manner.
 * Use:
 * `ControlResult theta_control`;
 */
#define ControlResult(X) ControlResult X = {.res = 0, .var = 0}

#define CIRCULAR 0b1
#define ROOF 0b10
#define NONE 0b0

// END types

// BEGIN Methods

/*
 * A SISO PI controller starting on cero with saturations.
 */
ControlResult pi_simple(float var, const Controler c, ControlResult past);

/*
 * An integrator with saturations
 */
ControlResult integ(float var, const Controler I, ControlResult past);

/*
 * Instantiates a new control result for a new controller
 */
ControlResult new_ControlResult();
// END Methods
#endif  // CONTROL_H
