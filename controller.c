#define S_FUNCTION_NAME  controller
#define S_FUNCTION_LEVEL 2

#define Tsampling 100e-6

// #include "simstruc.h"
#include "/home/david/.local/MATLAB/R2021b/simulink/include/simstruc.h"
#include "lib/transforms.h"

ThetaController theta;
Controller outer;
ParkController inner;

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 3)) return;// # puertos entrada
    //ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    //↓ una por entrada s, salida, dimensión
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortWidth(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    if (!ssSetNumOutputPorts(S,3)) return;
    //ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);
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
    //ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    // Tasa de muestreo ↓
    ssSetSampleTime(S, 0, Tsampling);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S) {
    theta = new_ThetaController(20, 15000, Tsampling);
    outer = new_Controller(0.2, 5, Tsampling);
    inner = new_ParkController(10, 400, Tsampling);
}
#endif /*  MDL_START */

static void mdlOutputs(SimStruct *S, int_T tid) {
    InputRealPtrsType pV = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType pI = ssGetInputPortRealSignalPtrs(S, 1);
    InputRealPtrsType pdc = ssGetInputPortRealSignalPtrs(S,2);
    real_T *y1 = ssGetOutputPortRealSignal(S,0);
    real_T *y2 = ssGetOutputPortRealSignal(S, 1);
    real_T *y3 = ssGetOutputPortRealSignal(S, 2);

    Normal V_n = {
        .a = *pV[0],
        .b = *pV[1],
        .c = *pV[2],
    };
    Normal I_n = {
        .a = *pI[0],
        .b = *pI[1],
        .c = *pI[2],
    };
    float V_dc = *pdc[0];

    Park V = park_tr(V_n, th_ang(theta));
    Park I = park_tr(I_n, th_ang(theta));

    InstantPower power = park_power(V, I);

    //BEGIN Outerloop
    const float V_dc_ref = 750;
    const float mag_ref = V_dc_ref * V_dc_ref / 2.0;
    float mag, mag_hat;
    mag = V_dc * V_dc / 2.0;
    mag_hat = mag_ref - mag;
    float p_ref = pi_simple(&outer,mag_hat);

    InstantPower pow_ref = {
        .P = p_ref,
        .Q = 5000
    };
    //END

    // BEGIN Innerloop

    const float L = 0.005;
    float gain = 1.0/(V.d * V.d + V.q * V.q);
    Park i_ref = {
        .d = gain * (V.d * pow_ref.P + V.q * pow_ref.Q),
        .q = gain * (V.q * pow_ref.P - V.d * pow_ref.Q),
    };
    Park i_hat = {
        .d = I.d - i_ref.d,
        .q = I.q - i_ref.q,
    };
    Park inner_res = actuate_Park(&inner, i_hat);

    Park comp = {
        .q = theta.ang_vel.res * L * I.d,
        .d = theta.ang_vel.res * L * I.q,
    };
    Park u_hat = {
        .d = inner_res.d + comp.d,
        .q = inner_res.q - comp.q,
    };
    Park U_p = {
      .d = u_hat.d + V.d,
      .q = u_hat.q + V.q
    };
    //END Innerloop

    Normal U = park_inv_tr(U_p, th_ang(theta));
    Normal i = park_inv_tr(I, th_ang(theta));

    theta_ctrl(&theta, V);

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
