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
} ControlResult;

#define CIRCULAR 0b1
#define ROOF 0b10
#define NONE 0b0

// END types

// BEGIN Methods

/*
 * A SISO PI controller starting on zero with saturations.
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

/*
 *
 */
float actuation(ControlResult a);

// END Methods


template <class T, class U>

/*
 * An abstract controller class to create a unified interface.
 *
 * This is thinking in terms of rust traits. It is abstract enough that anything
 * on the spirit of classic control strategies should be able to use it.
 */
class SimpleController{
public:
    /*
     * The control strategy
     */
    virtual U actuation(T var) = 0;
    /*
     * Passive read of the actuation value.
     */
    virtual U read() = 0;
};


/*
 * The ways in which things can possibly saturate
 */
enum class SaturationType { circular, roof, none };

struct SatParam{
    SaturationType type;
    float max;
    float min;
    SatParam() {
        type = SaturationType::none;
        max = 0;
        min = 0;
    };
};

/*
 * A SISO PI controller with saturation
 */
class PIController: SimpleController<double, double>{
protected:
    double Kp;
    double Ki;
    double T_samp;
    SatParam sat;
    double res;
    double var;
    void saturate();

public:
    /*
     * Initialize with specified saturation
     */
    PIController(double Kp,
                 double Ki,
                 double T_samp,
                 SatParam sat
                 );
    /*
     * With no saturation
     */
    PIController(double Kp,
                 double Ki,
                 double T_samp);
    /*
     * The actual PI controller
     */
    PIController();
    double actuation(double var);
    double read();
};

class Integrator: PIController{
public:
    Integrator(double T_samp, SatParam sat);
    double actuation(double var);
};


#endif  // CONTROL_H
