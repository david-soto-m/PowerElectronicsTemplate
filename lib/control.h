#ifndef CONTROL_H
#define CONTROL_H

// BEGIN types

/*
 * Ways in which a controller can saturate
 */
typedef enum SaturationTypes{
    roof,
    circular,
    none,
}SaturationTypes;


/*
 * All the info to saturate
 */
typedef struct Saturator{
    SaturationTypes sat;
    float max;
    float min;
}Saturator;

/*
 * Parameters for a PI controller
 */
typedef struct Controller{
    float Kp;
    float Ki;
    float T_samp;
    Saturator sat;
    float res;
    float var;
} ControlResult;



// END types

// BEGIN Methods

/*
 * A SISO PI controller starting on zero with saturations.
 */
ControlResult pi_simple(float var, Controller c);

/*
 * An integrator with saturations
 */
ControlResult integ(float var, Controller I);

/*
 * Instantiates a new control result for a new controller
 */
ControlResult new_ControlResult();

/*
 *
 */
float actuation(ControlResult a);

// END Methods
#endif  // CONTROL_H
