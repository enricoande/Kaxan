// Thrust force vector on a ROV
//
// Enrico Anderlini, University College London, e.anderlini@ucl.ac.uk
//
// Adapted from code by
// Gordon Parker
// Michigan Technological University
// Mechanical Engineering - Engineering Mechanics Deptartment
// Houghton, MI
//
// Created : 18 September 2017
//
// Version : 1.0

#define S_FUNCTION_NAME rov_thrust_simple
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) &&\
        mxIsDouble(pVal))

#define MYPI 3.14159265358979

#define YES 1
#define NO  0

// Manage the size of all the inputs, outputs, states and work vectors
// using the #define statements below. This off-loads the need for
// maintaining sizes in multiple spots below.

#define NP    4  // no. propellers
#define DOF   6  // no. degrees of freedom
#define O     4  // order of the polynomial fits + 1
        
// Parameters:
#define P_C   0  // coefficients 
#define P_T   1  // thrust allocation matrix
#define P_N   2  // number of parameters
        
// Continuous state indices:
#define C_N   0  // no. continuous states

// Real work vector indices:
#define RW_N NP  // size of real work vector
        
// Dynamic work vector indices:
#define DW_N  0  // size of dynamic work vector
        
// Integer work vector indices:
#define IW_N  0  // size of integer work vector

// Input indices:
#define I_V   0  // propellers' voltage
#define   I_VSIZE  NP // size of input port
#define I_N   1  // # of input ports

// Output indices:
#define O_T   0  // thrust vector
#define   O_TSIZE DOF // size of output port (6 DOF)
#define O_N   1  // # of output ports

// ---------------------  Support Functions  ------------------------------
// ************************************************************************
//  restoring_force: Function that returns the restoring force on the ROV.
// N.B.: For greater efficiency, a dynamic work vector is used.
// ************************************************************************
void propulsors_force(SimStruct *S)
{
    // Set a pointer to the real work vector:
    real_T *rw = ssGetRWork(S);
    // Get the polynomial coefficients:
    const real_T *C1D = mxGetPr(ssGetSFcnParam(S,P_C));
    real_T coeffs[2][O];
    memcpy(coeffs,C1D,O*2*sizeof(real_T));
    // Set a pointer to the input voltage:
    InputRealPtrsType V = ssGetInputPortRealSignalPtrs(S,I_V);
    
    // Compute the propulsors' force: 
    int_T i,j,k,l,m;
    for(k=0;k<2;k++) {
        for(i=0;i<2;i++) {
            l = k*2;
            m = l+i;
            rw[m] = 0.0;
            // Fit polynomial only outside of the dead zone:
            if (*V[m]<-0.5 || *V[m]>0.5) {
                for (j=0;j<O;j++) {
                    rw[m] += coeffs[k][j] * pow(*V[m],O-j-1.0); } } } }
}
// ------------------------------------------------------------------------
        
// ************************************************************************
// mdlCheckParameters: Method for checking parameter data types and sizes.
// Not required, but highly recommended to use it.
// ************************************************************************
#define MDL_CHECKPARAMETERS
#if defined(MDL_CHECKPARAMETERS)
static void mdlCheckParameters(SimStruct *S)
{
// Check 1st parameter: P_C
    {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_C)) != O*2 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_C)) ) {
        ssSetErrorStatus(S,"1st parameter, P_C, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
// Check 2nd parameter: P_T
    {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_T)) != NP*DOF || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_T)) ) {
        ssSetErrorStatus(S,"2nd parameter, P_T, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
}
#endif
 
// ************************************************************************
// mdlInitializeSize: Setup i/o and state sizes
// ************************************************************************
static void mdlInitializeSizes(SimStruct *S)
{
    //---------------------------------------------------------------------      
    //           *** P A R A M E T E R    S E T U P ***
    //---------------------------------------------------------------------  
    //   #  Description                                Units      Dim
    //---------------------------------------------------------------------      
    //   0. Coefficients of the polynomial fit          n/a        8
    //   1. Thrust allocation matrix                    n/a       24

    ssSetNumSFcnParams(S, P_N); // total number of parameters
  
    // Catch error made by user in giving parameter list to Simulink block
    // To make the mdlCheckParameters method active, it must be called as
    // shown below. This feature is not allowable in real-time, coder use,
    // so it is conditional on the MATLAB_MEX_FILE attribute. 
    #if defined(MATLAB_MEX_FILE)
    if( ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S) )
    {
        mdlCheckParameters(S);
        if(ssGetErrorStatus(S) != NULL) return;
    }
    else return; // parameter mismatch error
    #endif
  
    ssSetNumContStates(S,C_N);  // total number of continuous states           

    ssSetNumDiscStates(S,0);    // total number of discrete states

    // Set number of input ports:
    if (!ssSetNumInputPorts(S,I_N)) return;  
  
    // Set input port widths:
    ssSetInputPortWidth(S, I_V, I_VSIZE);

    // If you add new inputs, you must add an element to the list below to
    // indicate if the input is used directly to compute an output.  
    ssSetInputPortDirectFeedThrough(S, I_V, YES);
  
    // Specify number of output ports:
    if (!ssSetNumOutputPorts(S,O_N)) return; 

    // Specify output port widths:
    ssSetOutputPortWidth(S, O_T , O_TSIZE);

    // Set up work vectors:
    // If you need several arrays or 2D arrays of work vectors, then use
    // DWork.
    ssSetNumRWork(S, RW_N); 
    ssSetNumIWork(S, IW_N);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumDWork(S, DW_N);
    
    // Debugging:
//     int_T i,j;
//     // Get the polynomial coefficients:
//     const real_T *C1D = mxGetPr(ssGetSFcnParam(S,P_C));
//     real_T coeffs[2][O];
//     memcpy(coeffs,C1D,O*2*sizeof(real_T));
//     // Get the thrust allocation matrix:
//     const real_T *T1D = mxGetPr(ssGetSFcnParam(S,P_T));
//     real_T T[DOF][NP];
//     memcpy(T,T1D,NP*DOF*sizeof(real_T));
//     for(i=0;i<2;i++){
//         for(j=0;j<O;j++)
//             printf("%f\t", coeffs[i][j]);
//         printf("\n");}
//     for(i=0;i<DOF;i++){
//         for(j=0;j<NP;j++)
//             printf("%f\t", T[i][j]);
//         printf("\n");}

    // Set up sample times:
    ssSetNumSampleTimes(  S, 1);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE);
}

// ************************************************************************
// mdlInitializeSampleTimes: Set sample times for this s-fn. Modify this
// if you want to have the S-Fn called at interesting times. Lots of 
// documentation at MathWorks regarding how to manage this. 
// ************************************************************************
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

// ************************************************************************
// mdlInitializeConditions: Assign state ics, and other one-off actions.
// ************************************************************************
#undef MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S){}
#endif

// ************************************************************************
// mdlOutputs: Calc outputs at the start of each major integration step
// ************************************************************************
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Set a pointer to the outputs:
    real_T *yT   = ssGetOutputPortSignal(S,O_T);  // thrust vector
    // Get the thrust allocation matrix:
    const real_T *T1D = mxGetPr(ssGetSFcnParam(S,P_T));
    real_T T[DOF][NP];
    memcpy(T,T1D,NP*DOF*sizeof(real_T));
    // Compute the propulsors' force:
    propulsors_force(S);
    // Set a pointer to the real work vector:
    real_T *rw = ssGetRWork(S);
    
    // Compute the thrust force:
    int_T i,j;   // counters
    real_T tmp;  // temporary variable
    
    for(i=0;i<DOF;i++)
    {
        tmp = 0.0;
        for(j=0;j<NP;j++)
            tmp += T[i][j]*rw[j];
        yT[i] = tmp;
    }    
}

// ************************************************************************
// mdlUpdate: Update the discrete states
// ************************************************************************
#undef MDL_UPDATE 
#if defined(MDL_UPDATE)
static void mdlUpdate(SimStruct *S, int_T tid){}
#endif

// ************************************************************************
// mdlDerivatives: Calc state derivatives for integration
// ************************************************************************
#undef MDL_DERIVATIVES 
#if defined(MDL_DERIVATIVES)
static void mdlDerivatives(SimStruct *S){}
#endif

// ************************************************************************
// mdlTerminate: Clean up anything that needs it
// ************************************************************************
static void mdlTerminate(SimStruct *S) { }

// Here's some stuff that is all S-Functions at the end.
#ifdef  MATLAB_MEX_FILE    // Is this file being compiled as a MEX-file?
#include "simulink.c"      // MEX-file interface mechanism 
#else
#include "cg_sfun.h"       // Code generation registration function
#endif