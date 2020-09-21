/*
MPCPathFollowing_3v_IBR : A fast customized optimization solver.

Copyright (C) 2013-2020 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

/* Generated by FORCES PRO v3.1.0 on Sunday, September 20, 2020 at 2:21:42 PM */

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif

#ifndef MPCPathFollowing_3v_IBR_H
#define MPCPathFollowing_3v_IBR_H

/* DATA TYPE ------------------------------------------------------------*/
typedef double MPCPathFollowing_3v_IBR_float;

typedef double MPCPathFollowing_3v_IBRinterface_float;

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_MPCPathFollowing_3v_IBR
#define MISRA_C_MPCPathFollowing_3v_IBR (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_MPCPathFollowing_3v_IBR
#define RESTRICT_CODE_MPCPathFollowing_3v_IBR (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_MPCPathFollowing_3v_IBR
#define SET_PRINTLEVEL_MPCPathFollowing_3v_IBR    (1)
#endif

/* timing */
#ifndef SET_TIMING_MPCPathFollowing_3v_IBR
#define SET_TIMING_MPCPathFollowing_3v_IBR    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_MPCPathFollowing_3v_IBR			(500)	

/* scaling factor of line search (FTB rule) */
#define SET_FLS_SCALE_MPCPathFollowing_3v_IBR		(MPCPathFollowing_3v_IBR_float)(0.99)      

/* maximum number of supported elements in the filter */
#define MAX_FILTER_SIZE_MPCPathFollowing_3v_IBR	(500) 

/* maximum number of supported elements in the filter */
#define MAX_SOC_IT_MPCPathFollowing_3v_IBR			(4) 

/* desired relative duality gap */
#define SET_ACC_RDGAP_MPCPathFollowing_3v_IBR		(MPCPathFollowing_3v_IBR_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_MPCPathFollowing_3v_IBR		(MPCPathFollowing_3v_IBR_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_MPCPathFollowing_3v_IBR	(MPCPathFollowing_3v_IBR_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_MPCPathFollowing_3v_IBR	(MPCPathFollowing_3v_IBR_float)(1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_MPCPathFollowing_3v_IBR      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_MPCPathFollowing_3v_IBR (0)

/* wrong number of inequalities error */
#define INVALID_NUM_INEQ_ERROR_MPCPathFollowing_3v_IBR  (-4)

/* factorization error */
#define FACTORIZATION_ERROR_MPCPathFollowing_3v_IBR   (-5)

/* NaN encountered in function evaluations */
#define BADFUNCEVAL_MPCPathFollowing_3v_IBR  (-6)

/* no progress in method possible */
#define NOPROGRESS_MPCPathFollowing_3v_IBR   (-7)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_MPCPathFollowing_3v_IBR   (-11)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_MPCPathFollowing_3v_IBR  (-100)

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
    /* vector of size 480 */
    MPCPathFollowing_3v_IBR_float x0[480];

    /* vector of size 7 */
    MPCPathFollowing_3v_IBR_float xinit[7];

    /* vector of size 2000 */
    MPCPathFollowing_3v_IBR_float all_parameters[2000];


} MPCPathFollowing_3v_IBR_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
    /* vector of size 480 */
    MPCPathFollowing_3v_IBR_float alldata[480];


} MPCPathFollowing_3v_IBR_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    MPCPathFollowing_3v_IBR_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    MPCPathFollowing_3v_IBR_float res_ineq;

	/* norm of stationarity condition */
    MPCPathFollowing_3v_IBR_float rsnorm;

	/* max of all complementarity violations */
    MPCPathFollowing_3v_IBR_float rcompnorm;

    /* primal objective */
    MPCPathFollowing_3v_IBR_float pobj;	
	
    /* dual objective */
    MPCPathFollowing_3v_IBR_float dobj;	

    /* duality gap := pobj - dobj */
    MPCPathFollowing_3v_IBR_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    MPCPathFollowing_3v_IBR_float rdgap;		

    /* duality measure */
    MPCPathFollowing_3v_IBR_float mu;

	/* duality measure (after affine step) */
    MPCPathFollowing_3v_IBR_float mu_aff;
	
    /* centering parameter */
    MPCPathFollowing_3v_IBR_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    MPCPathFollowing_3v_IBR_float step_aff;
    
    /* step size (combined direction) */
    MPCPathFollowing_3v_IBR_float step_cc;    

	/* solvertime */
	MPCPathFollowing_3v_IBR_float solvetime;   

	/* time spent in function evaluations */
	MPCPathFollowing_3v_IBR_float fevalstime;  

} MPCPathFollowing_3v_IBR_info;







/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Sunday, September 20, 2020 2:21:45 PM */
/* User License expires on: (UTC) Monday, February 15, 2021 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Monday, February 15, 2021 10:00:00 PM (approx.) */
/* Solver Generation Request Id: 2d8c0faa-892a-45a8-aaaa-a487820a94de */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void (*MPCPathFollowing_3v_IBR_extfunc)(MPCPathFollowing_3v_IBR_float* x, MPCPathFollowing_3v_IBR_float* y, MPCPathFollowing_3v_IBR_float* lambda, MPCPathFollowing_3v_IBR_float* params, MPCPathFollowing_3v_IBR_float* pobj, MPCPathFollowing_3v_IBR_float* g, MPCPathFollowing_3v_IBR_float* c, MPCPathFollowing_3v_IBR_float* Jeq, MPCPathFollowing_3v_IBR_float* h, MPCPathFollowing_3v_IBR_float* Jineq, MPCPathFollowing_3v_IBR_float* H, solver_int32_default stage, solver_int32_default iterations);

extern solver_int32_default MPCPathFollowing_3v_IBR_solve(MPCPathFollowing_3v_IBR_params *params, MPCPathFollowing_3v_IBR_output *output, MPCPathFollowing_3v_IBR_info *info, FILE *fs, MPCPathFollowing_3v_IBR_extfunc evalextfunctions_MPCPathFollowing_3v_IBR);	





#ifdef __cplusplus
}
#endif

#endif
