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
} Controller;

// END types

// BEGIN Methods

/*
 * A SISO PI controller
 */
float pi_simple(Controller *c, float var);

/*
 * An integrator with saturations
 */
float integ(Controller *I, float var);

/*
 * Creates a new controller
 */
Controller new_Controller(float Kp, float Ki, float T_samp);

/*
 * Creates a new controller with saturations
 */
Controller new_Controller_w_sat(float Kp, float Ki, float T_samp, Saturator sat);

/*
 * Get the value of a controller
 */
float read(Controller a);


// END Methods
#endif  // CONTROL_H
