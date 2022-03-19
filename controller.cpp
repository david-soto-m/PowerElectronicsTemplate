#define S_FUNCTION_NAME  controller
#define S_FUNCTION_LEVEL 2

#define Tsampling 100e-6
#include <iostream>

// #include "simstruc.h"
#include "/home/david/.local/MATLAB/R2021b/simulink/include/simstruc.h"
#include "lib/transforms.h"

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
    if (!ssSetNumInputPorts(S, 3)) return;// # puertos entrada
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortWidth(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    if (!ssSetNumOutputPorts(S,3)) return;
    ssSetOutputPortWidth(S, 0, 3);
    ssSetOutputPortWidth(S, 1, 3);
    ssSetOutputPortWidth(S, 2, 2);

    ssSetNumSampleTimes(S, 1);
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, Tsampling);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S){}
#endif /*  MDL_START */

static void mdlOutputs(SimStruct *S, int_T tid) {

    static ThetaController theta_ctrl = ThetaController(20,15000,Tsampling);


    InputRealPtrsType pV = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType pI = ssGetInputPortRealSignalPtrs(S,1);
    InputRealPtrsType pdc = ssGetInputPortRealSignalPtrs(S, 2);
    real_T            *y1    = ssGetOutputPortRealSignal(S,0);
    real_T            *y2    = ssGetOutputPortRealSignal(S,1);
    real_T            *y3    = ssGetOutputPortRealSignal(S,2);
    double V_dc = *pdc[0];


    Park V = Park(Normal(*pV[0], *pV[1], *pV[2]), theta_ctrl.read());
    Park I = Park(Normal(*pI[0], *pI[1], *pI[2]), theta_ctrl.read());

    //BEGIN Outerloop
    const float V_dc_ref = 750;
    const float mag_ref = V_dc_ref * V_dc_ref / 2.0;
    float mag, mag_hat;
    mag = V_dc * V_dc / 2.0;
    mag_hat = mag_ref - mag;
    static PIController outer = PIController(0.2, 5, Tsampling);

    InstantPower pow_ref = {
        .P = outer.actuation(mag_hat),
        .Q = 5000
    };
    // END Outerloop

    //BEGIN Innerloop
    static ParkController inner_controller = ParkController(10, 400, Tsampling);

    float L = 0.005;
    float gain = 1.0/(V.d * V.d + V.q * V.q);
    Park i_ref = Park(
        gain * (V.d * pow_ref.P + V.q * pow_ref.Q),
        gain * (V.q * pow_ref.P - V.d * pow_ref.Q)
    );

    Park i_hat = I - i_ref;

    Park inner_res = inner_controller.actuation(i_hat);

    Park comp = Park(theta_ctrl.read_vel() * L * I.d,
                     theta_ctrl.read_vel() * L * I.q);

    Park u_hat = Park(inner_res.d + comp.d,
                      inner_res.q - comp.q);

    Park U_p = u_hat + V;

    //END Innerloop

    Normal i = I.to_normal(theta_ctrl.read());

    Normal U = U_p.to_normal(theta_ctrl.read());

    InstantPower power = V*I;
    theta_ctrl.actuation(V);

    y1[0] = U.a;
    y1[1] = U.b;
    y1[2] = U.c;
    y2[0] = i.a;
    y2[1] = i.b;
    y2[2] = i.c;
    y3[0] = power.P;
    y3[1] = power.Q;
}

static void mdlTerminate(SimStruct *S) {}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
