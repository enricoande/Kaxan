// Transpose of the generalised transformation matrix between inertial and 
// body-fixed frames in 3 DOF.
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

#define S_FUNCTION_NAME transp_tmat_3dof
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

#define DOF   3  // no. degrees of freedom
#define PHI   3
#define THETA 4
#define PSI   5        
        
// Parameters:
#define P_N   0  // number of parameters
        
// Continuous state indices:
#define C_N   0  // no. continuous states

// Real work vector indices:
#define RW_N  0  // size of real work vector
        
// Dynamic work vector indices:
#define DW_N  0  // size of dynamic work vector
        
// Integer work vector indices:
#define IW_N  0  // size of integer work vector

// Input indices:
#define I_X   0  // states
#define   I_XSIZE  12 // size of input port        
#define I_T   1  // thrust vector in the body-fixed frame in 4 DOF
#define   I_TSIZE DOF // size of input port
#define I_N   2  // # of input ports

// Output indices:
#define O_T   0  // thrust vector in the body-fixed frame in 4 DOF
#define   O_TSIZE DOF // size of output port (4 DOF)
#define O_N   1  // # of output ports
 
// ************************************************************************
// mdlInitializeSize: Setup i/o and state sizes
// ************************************************************************
static void mdlInitializeSizes(SimStruct *S)
{
    //---------------------------------------------------------------------      
    //           *** P A R A M E T E R    S E T U P ***
    //---------------------------------------------------------------------  

    ssSetNumSFcnParams(S, P_N); // total number of parameters
  
    ssSetNumContStates(S,C_N);  // total number of continuous states           

    ssSetNumDiscStates(S,0);    // total number of discrete states

    // Set number of input ports:
    if (!ssSetNumInputPorts(S,I_N)) return;  
  
    // Set input port widths:
    ssSetInputPortWidth(S, I_X, I_XSIZE);
    ssSetInputPortWidth(S, I_T, I_TSIZE);

    // If you add new inputs, you must add an element to the list below to
    // indicate if the input is used directly to compute an output.  
    ssSetInputPortDirectFeedThrough(S, I_X, YES);
    ssSetInputPortDirectFeedThrough(S, I_T, YES);
  
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
    real_T *t_bf = ssGetOutputPortSignal(S,O_T);  // thrust vector
    
    // Set a pointer to the inputs:
    InputRealPtrsType x = ssGetInputPortRealSignalPtrs(S,I_X);
    InputRealPtrsType t_if = ssGetInputPortRealSignalPtrs(S,I_T);
    
    // Compute the transpose of the translational and rotational 
    // transformation matrices:
    real_T R[3][3], phi, theta, psi;
    int_T i,j;   // counters
    
    phi   = *x[PHI];
    theta = *x[THETA];
    psi   = *x[PSI];
    
    R[0][0] = cos(psi)*cos(theta);
    R[0][1] = cos(theta)*sin(psi);
    R[0][2] = -sin(theta);
    R[1][0] = cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi);
    R[1][1] = cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta);
    R[1][2] = cos(theta)*sin(phi);
    R[2][0] = sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta);
    R[2][1] = cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi);
    R[2][2] = cos(theta)*cos(phi);
    
    // Output the thrust vector in 4 DOF in the body-fixed frame:
    for(i=0;i<DOF;i++) {
        t_bf[i] = 0.0;
        for(j=0;j<DOF;j++) {
            t_bf[i] += R[i][j]*(*t_if[j]); } }
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