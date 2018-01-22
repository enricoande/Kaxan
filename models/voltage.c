// Voltage in each thruster
//
// Enrico Anderlini, University College London, e.anderlini@ucl.ac.uk
//
// Adapted from code by
// Gordon Parker
// Michigan Technological University
// Mechanical Engineering - Engineering Mechanics Deptartment
// Houghton, MI
//
// Created : 22 January 2018
//
// Version : 1.0

#define S_FUNCTION_NAME voltage
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
#define NC    3  // no. coefficients used in the fit
        
// Parameters:
#define P_C   0  // coefficients for the fit
#define P_N   1  // number of parameters
        
// Continuous state indices:
#define C_N   0  // no. continuous states

// Real work vector indices:
#define RW_N NP  // size of real work vector
        
// Dynamic work vector indices:
#define DW_N  0  // size of dynamic work vector
        
// Integer work vector indices:
#define IW_N  0  // size of integer work vector

// Input indices:
#define I_T   0  // propellers' thrust
#define   I_TSIZE  NP // size of input port
#define I_N   1  // # of input ports

// Output indices:
#define O_V   0  // thrusters' voltage
#define   O_VSIZE  NP // size of output port (6 DOF)
#define O_N   1  // # of output ports
        
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
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_C)) != NP*NC || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_C)) ) {
        ssSetErrorStatus(S,"1st parameter, P_C, to S-function "
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
    //   0. Coefficients of the polynomial fit          n/a        12

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
    ssSetInputPortWidth(S, I_T, I_TSIZE);

    // If you add new inputs, you must add an element to the list below to
    // indicate if the input is used directly to compute an output.  
    ssSetInputPortDirectFeedThrough(S, I_T, YES);
  
    // Specify number of output ports:
    if (!ssSetNumOutputPorts(S,O_N)) return; 

    // Specify output port widths:
    ssSetOutputPortWidth(S, O_V , O_VSIZE);

    // Set up work vectors:
    // If you need several arrays or 2D arrays of work vectors, then use
    // DWork.
    ssSetNumRWork(S, RW_N); 
    ssSetNumIWork(S, IW_N);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumDWork(S, DW_N);
    
    // Debugging:
//     const real_T *C1D = mxGetPr(ssGetSFcnParam(S,P_C));
//     real_T coeffs[NP][NC];
//     memcpy(coeffs,C1D,NP*NC*sizeof(real_T));
//     int_T i,j;
//     for(i=0;i<NP;i++) {
//         for(j=0;j<NC;j++) {
//             printf("%f \t",coeffs[i][j]); }
//         printf("\n"); }

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
    int_T i,j;   // counters
    
    // Set a pointer to the outputs:
    real_T *V   = ssGetOutputPortSignal(S,O_V);  // thrusters' voltage
    // Set a pointer to the inputs:
    InputRealPtrsType thrust = ssGetInputPortRealSignalPtrs(S,I_T);   
    
    // Get the polynomial coefficients:
    const real_T *C1D = mxGetPr(ssGetSFcnParam(S,P_C));
    real_T coeffs[NP][NC];
    memcpy(coeffs,C1D,NP*NC*sizeof(real_T));
    
    // Get the voltage in each thruster:
    for(i=0;i<2;i++) {
        // 520 thrusters:
        if ((*thrust[i])<0) {
            V[i]=coeffs[0][0]*pow(-*thrust[i],coeffs[0][1])+coeffs[0][2];}
        else {
            V[i]=coeffs[1][0]*pow(*thrust[i],coeffs[1][1])+coeffs[1][2];}
        
        // 540 thrusters:
        j = i + 2;
        if ((*thrust[i])<0) {
            V[j]=coeffs[2][0]*pow(-*thrust[j],coeffs[2][1])+coeffs[2][2];}
        else {
            V[j]=coeffs[3][0]*pow(*thrust[j],coeffs[3][1])+coeffs[3][2];} }
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