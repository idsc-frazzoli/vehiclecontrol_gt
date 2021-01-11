/*
 * CasADi to FORCES Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2020. All rights reserved.
 *
 * This file is part of the FORCES client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "MPCPathFollowing_3v_N1/include/MPCPathFollowing_3v_N1.h"

#define casadi_real MPCPathFollowing_3v_N1_float


#include "MPCPathFollowing_3v_N1_casadi.h"
 

   

/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, MPCPathFollowing_3v_N1_float *data, MPCPathFollowing_3v_N1_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for( j=colidx[i]; j < colidx[i+1]; j++ )
        {
            out[i*nrow + row[j]] = data[j];
        }
    }
}




/* CasADi - FORCES interface */
extern void MPCPathFollowing_3v_N1_casadi2forces(MPCPathFollowing_3v_N1_float *x,        /* primal vars                                         */
                                 MPCPathFollowing_3v_N1_float *y,        /* eq. constraint multiplers                           */
                                 MPCPathFollowing_3v_N1_float *l,        /* ineq. constraint multipliers                        */
                                 MPCPathFollowing_3v_N1_float *p,        /* parameters                                          */
                                 MPCPathFollowing_3v_N1_float *f,        /* objective function (scalar)                         */
                                 MPCPathFollowing_3v_N1_float *nabla_f,  /* gradient of objective function                      */
                                 MPCPathFollowing_3v_N1_float *c,        /* dynamics                                            */
                                 MPCPathFollowing_3v_N1_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 MPCPathFollowing_3v_N1_float *h,        /* inequality constraints                              */
                                 MPCPathFollowing_3v_N1_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 MPCPathFollowing_3v_N1_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                            */
								 solver_int32_default iteration, /* iteration number of solver                          */
								 solver_int32_default threadID  /* Id of caller thread 								   */)
{
    /* CasADi input and output arrays */
    const MPCPathFollowing_3v_N1_float *in[4];
    MPCPathFollowing_3v_N1_float *out[7];
	

	/* Allocate working arrays for CasADi */
	MPCPathFollowing_3v_N1_float w[1141];
	
    /* temporary storage for casadi sparse output */
    MPCPathFollowing_3v_N1_float this_f;
    MPCPathFollowing_3v_N1_float nabla_f_sparse[24];
    MPCPathFollowing_3v_N1_float h_sparse[12];
    MPCPathFollowing_3v_N1_float nabla_h_sparse[45];
    MPCPathFollowing_3v_N1_float c_sparse[21];
    MPCPathFollowing_3v_N1_float nabla_c_sparse[87];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
    in[0] = x;
    in[1] = p; /* maybe should be made conditional */
    in[2] = l; /* maybe should be made conditional */     
    in[3] = y; /* maybe should be made conditional */


	if ((stage >= 0 && stage < 59))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		MPCPathFollowing_3v_N1_objective_1(in, out, NULL, w);
		if( nabla_f )
		{
			MPCPathFollowing_3v_N1_objective_1_sparsity(5, &nrow, &ncol, &colind, &row);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		out[1] = nabla_c_sparse;
		MPCPathFollowing_3v_N1_dynamics_1(in, out, NULL, w);
		if( c )
		{
			MPCPathFollowing_3v_N1_dynamics_1_sparsity(4, &nrow, &ncol, &colind, &row);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		if( nabla_c )
		{
			MPCPathFollowing_3v_N1_dynamics_1_sparsity(5, &nrow, &ncol, &colind, &row);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		MPCPathFollowing_3v_N1_inequalities_1(in, out, NULL, w);
		if( h )
		{
			MPCPathFollowing_3v_N1_inequalities_1_sparsity(4, &nrow, &ncol, &colind, &row);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if( nabla_h )
		{
			MPCPathFollowing_3v_N1_inequalities_1_sparsity(5, &nrow, &ncol, &colind, &row);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

	if ((stage >= 59 && stage < 60))
	{
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		MPCPathFollowing_3v_N1_objective_60(in, out, NULL, w);
		if( nabla_f )
		{
			MPCPathFollowing_3v_N1_objective_60_sparsity(5, &nrow, &ncol, &colind, &row);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		MPCPathFollowing_3v_N1_inequalities_60(in, out, NULL, w);
		if( h )
		{
			MPCPathFollowing_3v_N1_inequalities_60_sparsity(4, &nrow, &ncol, &colind, &row);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}

		if( nabla_h )
		{
			MPCPathFollowing_3v_N1_inequalities_60_sparsity(5, &nrow, &ncol, &colind, &row);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}

	}

         
    
    /* add to objective */
    if( f )
    {
        *f += this_f;
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif