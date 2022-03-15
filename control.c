#define S_FUNCTION_NAME  control
#define S_FUNCTION_LEVEL 2

#define Tsampling 100e-6//Tpwm en seg



// #include "simstruc.h"
#include "/home/david/.local/MATLAB/R2021b/simulink/include/simstruc.h"
#include "lib/transforms.h"

/*================*
 * Build checking *
 *================*/


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 2)) return;// # puertos entrada
    //ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    //↓ una por entrada s, salida, dimensión
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    if (!ssSetNumOutputPorts(S,3)) return;
    //ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetOutputPortWidth(S, 0, 2);
    ssSetOutputPortWidth(S, 1, 2);
    ssSetOutputPortWidth(S, 2, 2);

    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    //ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    // Tasa de muestreo ↓
    ssSetSampleTime(S, 0, Tsampling);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

#define MDL_START                      /* Change to #undef to remove function */
#if defined(MDL_START)
/* Function: mdlStart ==========================================================
 * Abstract:
 *
 */
// inicializaciones ↓
static void mdlStart(SimStruct *S)
{
}
#endif /*  MDL_START */


/*
 * Function: mdlOutputs =======================================================
 * Abstract:
 * Salidas
*/

static void mdlOutputs(SimStruct *S, int_T tid) {

    static ThetaResult theta = {
        .ang_vel = {
            .res = 0,
            .var = 0
        },
        .integ = {
            .res = 0,
            .var = 0,
        },
    };

    InputRealPtrsType pV = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType pI = ssGetInputPortRealSignalPtrs(S,1);
    real_T            *y1    = ssGetOutputPortRealSignal(S,0);
    real_T            *y2    = ssGetOutputPortRealSignal(S,1);
    real_T            *y3    = ssGetOutputPortRealSignal(S,2);

    ClarkePark V = {
        .a = *pV[0],
        .b = *pV[1],
        .c = *pV[2],
    };
    ClarkePark I = {
        .a = *pI[0],
        .b = *pI[1],
        .c = *pI[2],
    };
    V = park_tr(V, theta.integ.res);
    I = park_tr(I, theta.integ.res);

    InstantPower power = clarke_power(V, I);

    const Controler THETA_CTRL ={.Kp=25,.Ki=1000,.T_samp=Tsampling, .sat = NONE};
    theta = theta_ctrl(V,THETA_CTRL, theta);

    y1[0] = power.P;
    y1[1] = power.Q;
    y2[0] = V.d;
    y2[1] = V.q;
    y3[0] = I.d;
    y3[1] = I.q;
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
