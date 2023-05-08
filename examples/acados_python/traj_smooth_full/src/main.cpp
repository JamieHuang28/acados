// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_path_smoother.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     PATH_SMOOTHER_NX
#define NZ     PATH_SMOOTHER_NZ
#define NU     PATH_SMOOTHER_NU
#define NP     PATH_SMOOTHER_NP
#define NBX    PATH_SMOOTHER_NBX
#define NBX0   PATH_SMOOTHER_NBX0
#define NBU    PATH_SMOOTHER_NBU
#define NSBX   PATH_SMOOTHER_NSBX
#define NSBU   PATH_SMOOTHER_NSBU
#define NSH    PATH_SMOOTHER_NSH
#define NSG    PATH_SMOOTHER_NSG
#define NSPHI  PATH_SMOOTHER_NSPHI
#define NSHN   PATH_SMOOTHER_NSHN
#define NSGN   PATH_SMOOTHER_NSGN
#define NSPHIN PATH_SMOOTHER_NSPHIN
#define NSBXN  PATH_SMOOTHER_NSBXN
#define NS     PATH_SMOOTHER_NS
#define NSN    PATH_SMOOTHER_NSN
#define NG     PATH_SMOOTHER_NG
#define NBXN   PATH_SMOOTHER_NBXN
#define NGN    PATH_SMOOTHER_NGN
#define NY0    PATH_SMOOTHER_NY0
#define NY     PATH_SMOOTHER_NY
#define NYN    PATH_SMOOTHER_NYN
#define NH     PATH_SMOOTHER_NH
#define NPHI   PATH_SMOOTHER_NPHI
#define NHN    PATH_SMOOTHER_NHN
#define NPHIN  PATH_SMOOTHER_NPHIN
#define NR     PATH_SMOOTHER_NR


int main()
{

    path_smoother_solver_capsule *acados_ocp_capsule = path_smoother_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = PATH_SMOOTHER_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = path_smoother_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("path_smoother_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = path_smoother_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = path_smoother_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = path_smoother_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = path_smoother_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = path_smoother_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = path_smoother_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;

    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = 0;
    ubx0[0] = 0;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 0;
    ubx0[2] = 0;
    lbx0[3] = 0;
    ubx0[3] = 0;
    lbx0[4] = 0;
    ubx0[4] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    x_init[4] = 0.0;

    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    // set parameters
    double p[NP];
    p[0] = 3.1;

    for (int ii = 0; ii <= N; ii++)
    {
        path_smoother_acados_update_params(acados_ocp_capsule, ii, p, NP);
    }
  

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];


    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = path_smoother_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("path_smoother_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("path_smoother_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    path_smoother_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = path_smoother_acados_free(acados_ocp_capsule);
    if (status) {
        printf("path_smoother_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = path_smoother_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("path_smoother_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
