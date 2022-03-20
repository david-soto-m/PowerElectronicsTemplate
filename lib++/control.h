#ifndef CONTROL_H
#define CONTROL_H

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
    virtual T actuation(U var) = 0;
    /*
     * Passive read of the actuation value.
     */
    virtual T read() = 0;
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
    SatParam(SaturationType sat, double max, double min){
        type = sat;
        this->max = max;
        this->min = min;
    };
};

/*
 * A SISO PI controller with saturation
 */
class PIController: public SimpleController<double, double>{
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
/*
 * An integrator
 */
class Integrator: public PIController{
public:
    Integrator();
    Integrator(double T_samp, SatParam sat);
    double actuation(double var);
};


/*
 * It is frequent that you want to control two variables at the same time that
 * belong to the same object. (When controlling in the park or clarke domains)
 */
template <class T, class U>
class DualController : public SimpleController<T, U> {
public:
    PIController upper;
    PIController lower;
    virtual T actuation(U obj) = 0;
    T read() { return T(this->upper.read(), this->lower.read()); }
};

#endif  // CONTROL_H
